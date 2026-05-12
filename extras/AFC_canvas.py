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
    from extras.AFC_lane import AFCMoveWarning, MoveDirection, SpeedMode, AFCLane
except Exception:
    raise CONFIG_ERROR(
        ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc())
    )

try:
    from extras.AFC_unit import afcUnit
except Exception:
    raise CONFIG_ERROR(
        ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc())
    )


class afcCanvas(afcUnit):

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get("type", "canvas")
        self.prep_distance = config.getfloat("prep_distance", self.short_move_dis)
        self.prep_speed = config.getfloat("prep_speed", self.short_moves_speed)
        self.tool_load_sync_speed_offset = config.getfloat(
            "tool_load_sync_speed_offset", 5.0
        )
        self.tool_unload_sync_speed_offset = config.getfloat(
            "tool_unload_sync_speed_offset", 5.0
        )
        self.tool_unload_lane_extra_distance = config.getfloat(
            "tool_unload_lane_extra_distance", self.short_move_dis
        )
        self.tool_unload_lane_extra_speed = config.getfloat(
            "tool_unload_lane_extra_speed", self.short_moves_speed
        )
        self.tool_load_lane_extra_distance = config.getfloat(
            "tool_load_lane_extra_distance", None
        )
        self.pause_on_tangle = config.getboolean("pause_on_tangle", True)
        self.tangle_pin = config.get("tangle_pin", None)
        self.enable_9v = self._setup_output_pin(config, config.get("enable_9v_pin", None))
        self.enable_24v = self._setup_output_pin(config, config.get("enable_24v_pin", None))
        self._tangle_state = False

        if self.tangle_pin is not None:
            buttons = self.printer.load_object(config, "buttons")
            buttons.register_buttons([self.tangle_pin], self.tangle_callback)

    def handle_connect(self):
        super().handle_connect()
        self.logo = '<span class=success--text>CANVAS Ready</span>\n'
        self.logo_error = '<span class=error--text>CANVAS Not Ready</span>\n'
        
    def _setup_output_pin(self, config, pin_name):
        if pin_name is None:
            return None
        pins = self.printer.load_object(config, "pins")
        pin = pins.setup_pin("digital_out", pin_name)
        if hasattr(pin, "setup_start_value"):
            pin.setup_start_value(1.0, 1.0)
        return pin

    def prep_load(self, lane):
        self.logger.debug(f"prep_load: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_loading)

        getattr(lane, "move_with_odometer")(
            self.prep_distance,
            self.prep_speed,
            stop_condition=lambda: not bool(lane.prep_state),
        )

    def prep_post_load(self, lane):
        lane._load_state = lane.prep_state
        self.logger.debug(f"prep_post_load: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_ready)
        getattr(lane, "disengage_motors")(1.0)

    def eject_lane(self, lane):
        self.afc.error.AFC_error("CANVAS does not support ejecting lanes", pause=False)

    def move_to_hub(
        self,
        lane,
        dist,
        dir,
        use_homing=True,
        speed_mode=SpeedMode.LONG,
        assist_active=None,
    ):
        move_distance = abs(dist) * float(dir)
        if not use_homing:
            lane.move_advanced(move_distance, speed_mode)
            return True, 0, AFCMoveWarning.NONE

        speed_data = lane.get_speed_accel(speed_mode)
        speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
        sensor_active = bool(lane.get_toolhead_pre_sensor_state())
        target_state = True if move_distance >= 0 else False
        if sensor_active == target_state:
            return True, 0, AFCMoveWarning.NONE

        remaining = abs(dist)
        moved = 0.0
        chunk = max(getattr(lane, "short_move_dis", self.short_move_dis), 1.0)
        while remaining > 0:
            step = min(chunk, remaining)
            lane.move(step * float(dir), speed, lane.short_moves_accel, False)
            moved += step
            remaining -= step
            if bool(lane.get_toolhead_pre_sensor_state()) == target_state:
                return True, moved, AFCMoveWarning.NONE

        return False, moved, AFCMoveWarning.WARN

    def load_then_home(self, lane, distance, assist_active, endstop):
        speed_data = lane.get_speed_accel(SpeedMode.LONG)
        speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
        return getattr(lane, "_canvas_move_until")(endstop, distance, speed)

    def lane_not_ready(self, lane):
        self.logger.debug(f"lane_not_ready: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_not_ready)

    def lane_loaded(self, lane):
        self.logger.debug(f"lane_loaded: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_ready)

    def lane_unloaded(self, lane):
        self.logger.debug(f"lane_unloaded: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_off)
        lane._load_state = False

    def lane_loading(self, lane):
        self.logger.debug(f"lane_loading: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_loading)

    def lane_unloading(self, lane):
        self.logger.debug(f"lane_unloading: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_unloading)

    def lane_tool_loaded(self, lane):
        self.logger.debug(f"lane_tool_loaded: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_tool_loaded)
        lane.extruder_obj.set_status_led(self.afc.led_tool_loaded)

    def lane_tool_unloaded(self, lane):
        self.logger.debug(f"lane_tool_unloaded: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_ready)
        lane.extruder_obj.set_status_led(self.afc.led_tool_unloaded)

    def lane_tool_loaded_idle(self, lane):
        self.logger.debug(f"lane_tool_loaded_idle: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_tool_loaded_idle)
        lane.extruder_obj.set_status_led(self.afc.led_tool_loaded_idle)

    def lane_fault(self, lane):
        self.logger.debug(f"lane_fault: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_fault)

    def lane_illuminate_spool(self, lane):
        self.logger.debug(f"lane_illuminate_spool: {lane.name}")
        getattr(lane, "apply_canvas_led")(self.afc.led_spool_illum)

    def tangle_callback(self, eventtime, state):
        state = bool(state)
        if state and not self._tangle_state and self.pause_on_tangle:
            if self.function.is_printing():
                self.afc.error.AFC_error(
                    "CANVAS tangle detected on unit {}".format(self.name),
                    pause=True,
                )
        self._tangle_state = state

    def system_Test(
        self,
        cur_lane: AFCLane,
        delay: float,
        assignTcmd: bool,
        enable_movement: bool,
    ) -> bool:
        # For now, ignore movement checks
        msg = ""
        succeeded = True

        if not cur_lane.prep_state:
            self.lane_unloaded(cur_lane)
            msg = "EMPTY READY FOR SPOOL"
        else:
            self.lane_loaded(cur_lane)
            msg = "FILAMENT PRESENT"

            if (cur_lane.tool_loaded
                and cur_lane.extruder_obj is not None
                and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
                cur_lane.sync_to_extruder()
                if self.afc.current == cur_lane.name:
                    self.lane_tool_loaded(cur_lane)
                else:
                    self.lane_tool_loaded_idle(cur_lane)
                msg += " in ToolHead"

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)

        cur_lane.send_lane_data()
        cur_lane.do_enable(False)
        self.logger.info(f"{cur_lane.name} tool cmd: {cur_lane.map:3} {msg}")
        cur_lane.set_afc_prep_done()
        return succeeded



def load_config_prefix(config):
    return afcCanvas(config)