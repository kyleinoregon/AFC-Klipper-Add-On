from __future__ import annotations

from unittest.mock import MagicMock, call

from extras.AFC_canvas_lane import AFCCanvasLane
from extras.AFC_lane import AFCHomingPoints, AFCMoveWarning, SpeedMode
from tests.conftest import MockAFC, MockLogger, MockPrinter, MockReactor


def _make_canvas_lane(name="lane1"):
    lane = AFCCanvasLane.__new__(AFCCanvasLane)
    afc = MockAFC()
    afc.tool_cut = True
    afc.park = True
    afc.form_tip = False
    afc.tool_cut_cmd = "AFC_CUT"
    afc.park_cmd = "AFC_PARK"
    afc.form_tip_cmd = "AFC"
    afc.tool_max_unload_attempts = 4
    afc._check_extruder_temp = MagicMock(return_value=False)
    afc.move_e_pos = MagicMock()
    afc.error.handle_lane_failure = MagicMock()
    reactor = MockReactor()
    printer = MockPrinter(afc=afc)

    lane.printer = printer
    lane.afc = afc
    lane.logger = MockLogger()
    lane.reactor = reactor
    lane.name = name
    lane.unit_obj = MagicMock()
    lane.unit_obj.select_lane = MagicMock()
    lane.unit_obj.move_to_hub = MagicMock(return_value=(True, 10.0, AFCMoveWarning.NONE))
    lane.unit_obj.lane_unloading = MagicMock()
    lane.unit_obj.cutter_sensor_state = False
    lane.canvas_motor = MagicMock()
    lane.red_led_pin = MagicMock()
    lane.white_led_pin = MagicMock()
    lane.short_move_dis = 5.0
    lane.short_moves_speed = 20.0
    lane.short_moves_accel = 400.0
    lane.long_moves_speed = 80.0
    lane.long_moves_accel = 400.0
    lane.tool_load_sync_speed_offset = 4.0
    lane.tool_unload_sync_speed_offset = 6.0
    lane.tool_unload_lane_extra_distance = 7.0
    lane.tool_unload_lane_extra_speed = 18.0
    lane.disengage_distance = 2.0
    lane.dist_hub = 100.0
    lane.loaded_to_hub = False
    lane._load_state = True
    lane.buffer_obj = MagicMock()
    lane.endstops = {}
    lane.only_lane = True
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = "extruder"
    lane.extruder_obj.tool_load_speed = 25.0
    lane.extruder_obj.tool_unload_speed = 20.0
    lane.extruder_obj.tool_stn = 72.0
    lane.extruder_obj.tool_stn_unload = 40.0
    lane.extruder_obj.tool_sensor_after_extruder = 12.0
    lane.extruder_obj.tool_end = None
    lane.extruder_obj.tool_end_state = False
    lane.extruder_obj.estats = MagicMock()
    lane.get_speed_accel = MagicMock(side_effect=lambda mode: (30.0, 300.0))
    lane.move_advanced = MagicMock()
    lane.disable_buffer = MagicMock()
    lane.select_lane = MagicMock()
    lane.do_enable = AFCCanvasLane.do_enable.__get__(lane, AFCCanvasLane)
    lane._last_odometer_state = False
    lane.odometer_count = 0
    lane.last_odometer_eventtime = None
    lane.odometer_poll_interval = AFCCanvasLane.DEFAULT_ODOMETER_POLL_INTERVAL
    lane.odometer_mm_per_pulse = 0.5
    lane.drv8833_object_name = "drv8833 lane1"
    lane.connect_done = True
    lane.unit = "CANVAS_1"
    lane.hub = "hub"
    lane.extruder_name = "extruder"
    lane.buffer_name = None
    lane.index = 1
    lane.map = "T0"
    lane.prep_state = True
    lane.tool_loaded = False
    lane._material = None
    lane.remember_spool = False
    lane.spool_id = None
    lane.color = None
    lane.weight = 0
    lane.extruder_temp = 0
    lane.bed_temp = 0
    lane.runout_lane = None
    lane.status = "Loaded"
    lane.td1_data = {}
    lane._selector_state = None
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)
    lane.buffer_status = MagicMock(return_value=None)
    lane._set_gpio_pin = AFCCanvasLane._set_gpio_pin.__get__(lane, AFCCanvasLane)
    return lane


def test_move_translates_signed_direction():
    lane = _make_canvas_lane()

    AFCCanvasLane.move(lane, -12.0, 30.0, 400.0)

    lane.unit_obj.select_lane.assert_called_once_with(lane)
    lane.canvas_motor.drv8833_move.assert_called_once_with(
        30.0, -12.0, wait_for_completion=True
    )


def test_canvas_feed_keeps_extruder_moving_until_canvas_completes():
    lane = _make_canvas_lane()
    lane.canvas_motor.active = True

    def move_e_side_effect(*args, **kwargs):
        if lane.afc.move_e_pos.call_count >= 3:
            lane.canvas_motor.active = False

    lane.afc.move_e_pos.side_effect = move_e_side_effect

    AFCCanvasLane._move_canvas_with_extruder_feed(
        lane, 12.0, 20.0, 25.0, "CANVAS tool load extra move"
    )

    lane.canvas_motor.drv8833_move.assert_called_once_with(
        20.0, 12.0, wait_for_completion=False
    )
    assert lane.afc.move_e_pos.call_args_list == [
        call(1.0, 25.0, "CANVAS tool load extra move", wait_tool=True),
        call(1.0, 25.0, "CANVAS tool load extra move", wait_tool=True),
        call(1.0, 25.0, "CANVAS tool load extra move", wait_tool=True),
    ]
    lane.canvas_motor.drv8833_set_speed.assert_called_once_with(0.0)


def test_do_enable_false_stops_motor():
    lane = _make_canvas_lane()

    lane.do_enable(False)

    lane.canvas_motor.drv8833_set_speed.assert_called_once_with(0.0)


def test_set_extruder_assist_applies_signed_offset():
    lane = _make_canvas_lane()

    AFCCanvasLane.set_extruder_assist(lane, -25.0, 5.0)

    lane.canvas_motor.drv8833_set_speed.assert_called_once_with(-20.0)


def test_move_to_without_homing_returns_success():
    lane = _make_canvas_lane()

    success, moved, warn = AFCCanvasLane.move_to(lane, 30.0, SpeedMode.LONG, use_homing=False)

    assert success is True
    assert moved == 0
    assert warn == AFCMoveWarning.NONE
    lane.move_advanced.assert_called_once_with(30.0, SpeedMode.LONG)


def test_move_to_with_homing_stops_on_sensor():
    lane = _make_canvas_lane()
    lane.get_toolhead_pre_sensor_state = MagicMock(side_effect=[False, True])
    lane.move = MagicMock()

    success, moved, warn = AFCCanvasLane.move_to(
        lane, 20.0, SpeedMode.SHORT, endstop=AFCHomingPoints.TOOL, use_homing=True
    )

    assert success is True
    assert moved == 5.0
    assert warn == AFCMoveWarning.NONE


def test_odometer_counts_active_edges_only():
    lane = _make_canvas_lane()

    AFCCanvasLane.odometer_callback(lane, 1.0, True)
    AFCCanvasLane.odometer_callback(lane, 2.0, True)
    AFCCanvasLane.odometer_callback(lane, 3.0, False)
    AFCCanvasLane.odometer_callback(lane, 4.0, True)

    assert lane.odometer_count == 2
    assert lane.get_odometer_distance() == 1.0


def test_move_with_odometer_uses_set_speed_and_stops_at_target():
    lane = _make_canvas_lane()
    lane.reset_odometer = MagicMock()
    lane.unit_obj.select_lane = MagicMock()
    lane.get_odometer_distance = MagicMock(side_effect=[0.0, 0.5, 1.0, 1.5, 2.0])
    lane.reactor.monotonic = MagicMock(side_effect=[0.0, 0.0, 0.01, 0.01, 0.02, 0.02, 0.03])
    lane.reactor.pause = MagicMock()

    moved = AFCCanvasLane.move_with_odometer(lane, 2.0, 25.0)

    assert moved == 2.0
    lane.reset_odometer.assert_called_once_with()
    lane.unit_obj.select_lane.assert_called_once_with(lane)
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[0] == call(25.0)
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[-1] == call(0.0)


def test_move_with_odometer_stops_early_when_condition_is_met():
    lane = _make_canvas_lane()
    lane.reset_odometer = MagicMock()
    lane.unit_obj.select_lane = MagicMock()
    lane.get_odometer_distance = MagicMock(return_value=0.5)
    lane.reactor.monotonic = MagicMock(return_value=0.0)
    lane.reactor.pause = MagicMock()
    stop_condition = MagicMock(side_effect=[False, True])

    moved = AFCCanvasLane.move_with_odometer(
        lane, 3.0, 20.0, stop_condition=stop_condition
    )

    assert moved == 0.5
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[0] == call(20.0)
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[-1] == call(0.0)


def test_apply_canvas_led_maps_red_and_white():
    lane = _make_canvas_lane()
    lane.red_led_pin.get_mcu.return_value.estimated_print_time.return_value = 0.0
    lane.red_led_pin.get_mcu.return_value.min_schedule_time.return_value = 0.0
    lane.white_led_pin.get_mcu.return_value.estimated_print_time.return_value = 0.0
    lane.white_led_pin.get_mcu.return_value.min_schedule_time.return_value = 0.0

    AFCCanvasLane.apply_canvas_led(lane, "1,0,0,1")

    lane.red_led_pin.set_digital.assert_called_once_with(0.0, 1.0)
    lane.white_led_pin.set_digital.assert_called_once_with(0.0, 1.0)


def test_custom_load_runs_hub_move_and_assisted_extruder_move():
    lane = _make_canvas_lane()
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)

    AFCCanvasLane.cmd_AFC_CANVAS_TOOL_LOAD(lane, MagicMock())

    lane.unit_obj.move_to_hub.assert_called_once()
    lane.afc.gcode.run_script_from_command.assert_called_once_with(
        "AFC_PARK EXTRUDER=extruder"
    )
    lane.afc.move_e_pos.assert_called_once_with(72.0, 25.0, "CANVAS tool stn", wait_tool=False)
    assert lane.loaded_to_hub is True
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[-1] == call(0.0)


def test_custom_load_stops_when_cutter_sensor_engages():
    lane = _make_canvas_lane()
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)

    def pause_side_effect(until):
        lane.unit_obj.cutter_sensor_state = True

    lane.reactor.pause = MagicMock(side_effect=pause_side_effect)

    AFCCanvasLane.cmd_AFC_CANVAS_TOOL_LOAD(lane, MagicMock())

    lane.afc.move_e_pos.assert_not_called()
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[-1] == call(0.0)
    assert lane.loaded_to_hub is False


def test_custom_unload_runs_macros_then_retracts():
    lane = _make_canvas_lane()
    lane.get_toolhead_pre_sensor_state = MagicMock(side_effect=[True, False])
    lane.move = MagicMock()
    lane.disengage_motors = MagicMock()

    AFCCanvasLane.cmd_AFC_CANVAS_TOOL_UNLOAD(lane, MagicMock())

    assert lane.afc.gcode.run_script_from_command.call_args_list[:2] == [
        call("AFC_CUT EXTRUDER=extruder"),
        call("AFC_PARK EXTRUDER=extruder"),
    ]
    assert lane.afc.move_e_pos.call_args_list[:3] == [
        call(-2, 20.0, "Quick Pull", wait_tool=False),
        call(-40.0, 20.0, "CANVAS tool unload", wait_tool=True),
        call(-12.0, 20.0, "CANVAS after extruder", wait_tool=True),
    ]
    lane.move.assert_any_call(-5.0, 20.0, 400.0, False)
    lane.move.assert_any_call(-7.0, 18.0, 400.0, False)
    lane.disengage_motors.assert_called_once_with(-1.0)
    assert lane.loaded_to_hub is False
