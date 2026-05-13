from __future__ import annotations

import traceback

import configfile

CONFIG_ERROR = getattr(configfile, "error", Exception)

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise CONFIG_ERROR(
        "Error when trying to import AFC_utils.ERROR_STR\n{}".format(
            traceback.format_exc()
        )
    )

try:
    from extras.AFC_lane import (
        AFCHomingPoints,
        AFCLane,
        AFCMoveWarning,
    )
except Exception:
    raise CONFIG_ERROR(
        ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc())
    )


class AFCCanvasLane(AFCLane):
    cmd_AFC_CANVAS_TOOL_LOAD_help = "CANVAS-specific tool load for a lane"
    cmd_AFC_CANVAS_TOOL_UNLOAD_help = "CANVAS-specific tool unload for a lane"
    DEFAULT_ODOMETER_POLL_INTERVAL = 0.05
    GPIO_PIN_MIN_TIME = 2

    def __init__(self, config):
        super().__init__(config)
        self.drv8833_object_name = config.get("drv8833", None)
        if self.drv8833_object_name is None:
            raise CONFIG_ERROR(
                "drv8833_object must be configured in [{}]".format(config.get_name())
            )

        self.canvas_motor = self.printer.lookup_object("drv8833 " + self.drv8833_object_name, None)
        if self.canvas_motor is None:
            raise CONFIG_ERROR(
                "Unable to find CANVAS motor object '{}' for lane {}".format(
                    self.drv8833_object_name, self.name
                )
            )

        pins = self.printer.lookup_object("pins")
        self.red_led_pin = self._setup_led_pin(pins, config.get("led_red_pin", None))
        self.white_led_pin = self._setup_led_pin(pins, config.get("led_white_pin", None))

        self.odometer_pin = config.get("odometer_pin", None)
        self.odometer_mm_per_pulse = config.getfloat("odometer_resolution", None)
        self.odometer_poll_interval = config.getfloat(
            "odometer_poll_interval", self.DEFAULT_ODOMETER_POLL_INTERVAL, minval=0.01
        )
        self.odometer_count = 0
        self.last_odometer_eventtime = None
        self._last_odometer_state = False
        if self.odometer_pin is not None:
            buttons = self.printer.load_object(config, "buttons")
            buttons.register_buttons([self.odometer_pin], self.odometer_callback)

        self.disengage_distance = config.getfloat("disengage_distance", 1.0)

        if self.custom_load_cmd is None:
            self.custom_load_cmd = "AFC_CANVAS_TOOL_LOAD LANE={}".format(self.name)
        if self.custom_unload_cmd is None:
            self.custom_unload_cmd = "AFC_CANVAS_TOOL_UNLOAD LANE={}".format(self.name)

        self.afc.gcode.register_mux_command(
            "AFC_CANVAS_TOOL_LOAD",
            "LANE",
            self.name,
            self.cmd_AFC_CANVAS_TOOL_LOAD,
            desc=self.cmd_AFC_CANVAS_TOOL_LOAD_help,
        )
        self.afc.gcode.register_mux_command(
            "AFC_CANVAS_TOOL_UNLOAD",
            "LANE",
            self.name,
            self.cmd_AFC_CANVAS_TOOL_UNLOAD,
            desc=self.cmd_AFC_CANVAS_TOOL_UNLOAD_help,
        )

        self._register_frontend_compat_aliases()
        self._load_state = False

    def _setup_led_pin(self, pins, pin_name):
        if pin_name is None:
            return None
        pin = pins.setup_pin("digital_out", pin_name)
        pin.setup_start_value(0.0, 0.0)
        pin.setup_max_duration(0.0)
        pin.last_set_time = 0.0
        return pin

    def _set_gpio_pin(self, pin, enabled):
        if pin is None:
            return
        
        print_time = pin.get_mcu().estimated_print_time(self.reactor.monotonic()) + pin.get_mcu().min_schedule_time() + 0.1
        print_time = max(print_time, pin.last_set_time + 0.2)
        pin.last_set_time = print_time

        pin.set_digital(
            print_time,
            1 if enabled else 0,
        )

    def _register_frontend_compat_aliases(self) -> None:
        for alias in (f"AFC_lane {self.name}", f"AFC_stepper {self.name}"):
            if alias == self.fullname:
                continue

            existing = self.printer.lookup_object(alias, None)
            if existing not in (None, self):
                continue

            add_object = getattr(self.printer, "add_object", None)
            if callable(add_object):
                try:
                    add_object(alias, self)
                    continue
                except Exception:
                    pass

            for registry_name in ("_objects", "objects"):
                registry = getattr(self.printer, registry_name, None)
                if isinstance(registry, dict):
                    registry.setdefault(alias, self)
                    break

    def move(self, distance, speed, accel, assist_active=False):
        self.unit_obj.select_lane(self)
        if distance == 0:
            return
        self.canvas_motor.drv8833_move(speed, distance)

    def prep_callback(self, eventtime, state):
        # TODO: This doesn't work all the time
        if not self._afc_prep_done:
            self._load_state = state

        super().prep_callback(eventtime, state)

    def do_enable(self, enable):
        if not enable:
            self.canvas_motor.drv8833_set_speed(0.0)

    def set_extruder_assist(self, extruder_speed, speed_offset):
        direction = 1.0 if extruder_speed >= 0 else -1.0
        assist_speed = max(abs(extruder_speed) - max(speed_offset, 0.0), 0.0)
        self.canvas_motor.drv8833_set_speed(direction * assist_speed)

    def _canvas_endstop_state(self, endstop):
        if endstop in (
            AFCHomingPoints.HUB,
            AFCHomingPoints.TOOL,
            AFCHomingPoints.TOOL_START,
            self.hub_endstop_name,
            self.tool_endstop_name,
        ):
            return bool(self.get_toolhead_pre_sensor_state())
        if endstop in (AFCHomingPoints.LOAD, self.load_es):
            return bool(self.raw_load_state)
        if endstop == AFCHomingPoints.BUFFER:
            return bool(getattr(self.buffer_obj, "advance_state", False))
        if endstop == AFCHomingPoints.BUFFER_TRAIL:
            return bool(getattr(self.buffer_obj, "trailing_state", False))
        return False

    def _canvas_move_until(self, endstop, distance, speed):
        desired_state = True if distance >= 0 else False
        remaining = abs(distance)
        moved = 0.0
        chunk = max(getattr(self, "short_move_dis", 1.0), 1.0)
        if self._canvas_endstop_state(endstop) == desired_state:
            return True, 0.0, AFCMoveWarning.NONE

        while remaining > 0:
            step = min(chunk, remaining)
            self.move(step if distance >= 0 else -step, speed, self.short_moves_accel, False)
            moved += step
            remaining -= step
            if self._canvas_endstop_state(endstop) == desired_state:
                return True, moved, AFCMoveWarning.NONE

        return False, moved, AFCMoveWarning.WARN

    def move_to(
        self,
        distance,
        speed_mode,
        endstop=AFCHomingPoints.NONE,
        assist_active=None,
        use_homing=True,
    ):
        if not use_homing:
            self.move_advanced(distance, speed_mode)
            return True, 0, AFCMoveWarning.NONE

        speed_data = self.get_speed_accel(speed_mode)
        speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
        return self._canvas_move_until(endstop, distance, speed)

    def apply_canvas_led(self, color_string):
        channels = [item.strip() for item in str(color_string).split(",")]
        while len(channels) < 4:
            channels.append("0")
        red_on = float(channels[0]) > 0
        white_on = float(channels[3]) > 0
        self._set_gpio_pin(self.red_led_pin, red_on)
        self._set_gpio_pin(self.white_led_pin, white_on)

    def disengage_motors(self, direction):
        if self.disengage_distance <= 0:
            self.do_enable(False)
            return
        self.reactor.pause(self.reactor.monotonic() + 0.5)
        release_distance = -abs(self.disengage_distance) if direction >= 0 else abs(self.disengage_distance)
        self.move(release_distance, 30.0, self.short_moves_accel, False)
        self.do_enable(False)

    def odometer_callback(self, eventtime, state):
        state = bool(state)
        if state and not self._last_odometer_state:
            self.odometer_count += 1
            self.last_odometer_eventtime = eventtime
        self._last_odometer_state = state

    def reset_odometer(self):
        self.odometer_count = 0
        self.last_odometer_eventtime = None
        self._last_odometer_state = False

    def get_odometer_distance(self):
        if self.odometer_mm_per_pulse is None:
            return 0.0
        return self.odometer_count * self.odometer_mm_per_pulse

    def move_with_odometer(self, distance, speed, stop_condition=None):
        target_distance = abs(distance)
        if target_distance == 0:
            return 0.0

        if self.odometer_mm_per_pulse is None:
            raise CONFIG_ERROR(
                "odometer_mm_per_pulse must be configured for {} to use move_with_odometer".format(
                    self.name
                )
            )

        signed_speed = abs(speed) if distance >= 0 else -abs(speed)
        moved = 0.0
        timeout = (target_distance / max(abs(speed), 1.0)) * 5.0 + 1.0
        poll_interval = max(
            getattr(
                self,
                "odometer_poll_interval",
                self.DEFAULT_ODOMETER_POLL_INTERVAL,
            ),
            0.01,
        )
        deadline = self.reactor.monotonic() + timeout

        self.reset_odometer()
        self.unit_obj.select_lane(self)

        try:
            self.canvas_motor.drv8833_set_speed(signed_speed)
            while moved < target_distance:
                if stop_condition is not None and stop_condition():
                    break
                moved = self.get_odometer_distance()
                if moved >= target_distance:
                    break
                now = self.reactor.monotonic()
                if now >= deadline:
                    raise CONFIG_ERROR(
                        "Timed out waiting for odometer movement on {}".format(
                            self.name
                        )
                    )
                self.reactor.pause(now + poll_interval)
                moved = self.get_odometer_distance()
        finally:
            self.canvas_motor.drv8833_set_speed(0.0)

        return min(moved, target_distance)

    def _run_unload_macros(self):
        if self.afc.tool_cut:
            self.extruder_obj.estats.increase_cut_total()
            self.afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.afc.tool_cut_cmd, self.extruder_obj.name)
            )
            if self.afc.park:
                self.afc.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(self.afc.park_cmd, self.extruder_obj.name)
                )

        if self.afc.form_tip:
            if self.afc.park:
                self.afc.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(self.afc.park_cmd, self.extruder_obj.name)
                )
            if self.afc.form_tip_cmd == "AFC":
                self.printer.lookup_object("AFC_form_tip").tip_form()
            else:
                self.afc.gcode.run_script_from_command(self.afc.form_tip_cmd)

    def _assist_extruder_move(self, distance, speed, speed_offset, label):
        try:
            self.set_extruder_assist(speed if distance >= 0 else -speed, speed_offset)
            self.afc.move_e_pos(distance, speed, label, wait_tool=True)
        finally:
            self.canvas_motor.drv8833_set_speed(0.0)

    def _cutter_sensor_engaged(self):
        return bool(getattr(self.unit_obj, "cutter_sensor_state", False))

    def cmd_AFC_CANVAS_TOOL_LOAD(self, gcmd):
        self.select_lane()
        self.afc._check_extruder_temp(self)

        if self._cutter_sensor_engaged():
            self.logger.error(f"CANVAS tool load aborted because the cutter sensor is engaged")

        if self.afc.park:
            self.afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.afc.park_cmd, self.extruder_obj.name)
            )

        if not self.get_toolhead_pre_sensor_state():
            start = self.reactor.monotonic()
            self.canvas_motor.drv8833_set_speed(self.short_moves_speed)
            while not self.get_toolhead_pre_sensor_state():
                now = self.reactor.monotonic()
                if now - start > 15.0:
                    self.canvas_motor.drv8833_set_speed(0.0)
                    self.logger.error(f"CANVAS load failed to reach the shared toolhead sensor for {self.name} within 15 seconds.")
                    return
                self.reactor.pause(now + 0.005)
            
            self.canvas_motor.drv8833_set_speed(0.0)
            
        self.loaded_to_hub = True

        if self.hub_obj and self.hub_obj.afc_bowden_length > 0:
            self.afc.move_e_pos(self.hub_obj.afc_bowden_length, self.extruder_obj.tool_load_speed, "CANVAS tool load extra move", wait_tool=False)
            self.move(self.hub_obj.afc_bowden_length, self.short_moves_speed, self.short_moves_accel)

        self.reset_odometer()

        if self.extruder_obj.tool_stn > 0:
            self.afc.move_e_pos(self.extruder_obj.tool_stn, self.extruder_obj.tool_load_speed, "CANVAS tool load", wait_tool=True)

        if self.odometer_count <= 5:
            self.logger.error(f"Odometer count after load is {self.odometer_count}, which may mean the filamnet was not grabbed by the extruder. Attempting to unload.")
            if self.extruder_obj.tool_stn_unload > 0:
                self.afc.move_e_pos(-self.extruder_obj.tool_stn_unload, self.extruder_obj.tool_unload_speed, "CANVAS tool unload", wait_tool=True)

            if self.hub_obj and self.hub_obj.afc_unload_bowden_length > 0:
                self.move_with_odometer(-self.hub_obj.afc_unload_bowden_length, self.long_moves_speed)

            self.disengage_motors(-1.0)
            return
        
        # TODO: Maybe also check the pressure sensor?
        
        self.disengage_motors(1.0)

    def cmd_AFC_CANVAS_TOOL_UNLOAD(self, gcmd):
        self.select_lane()
        self.afc._check_extruder_temp(self)
        self.disable_buffer()
        self.unit_obj.lane_unloading(self)
        self._run_unload_macros()

        if self.extruder_obj.tool_stn_unload > 0:
            self.afc.move_e_pos(-self.extruder_obj.tool_stn_unload, self.extruder_obj.tool_unload_speed, "CANVAS tool unload", wait_tool=True)

        if self.hub_obj and self.hub_obj.afc_unload_bowden_length > 0:
            self.move_with_odometer(-self.hub_obj.afc_unload_bowden_length, self.long_moves_speed)

        if self.get_toolhead_pre_sensor_state():
            raise gcmd.error(f"CANVAS unload failed to clear the shared toolhead sensor for {self.name} after extruder move")

        self.loaded_to_hub = False
        self.disengage_motors(-1.0)

    def get_status(self, eventtime=None, save_to_file=False):
        response = super().get_status(eventtime=eventtime, save_to_file=save_to_file)
        if not response:
            return response
        response["canvas_motor"] = self.drv8833_object_name
        response["odometer_count"] = self.odometer_count
        response["odometer_distance"] = self.get_odometer_distance()
        return response

def load_config_prefix(config):
    return AFCCanvasLane(config)