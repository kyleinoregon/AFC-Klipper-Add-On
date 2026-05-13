from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_canvas import afcCanvas
from extras.AFC_lane import AFCMoveWarning, MoveDirection, SpeedMode, AFCHomingPoints
from tests.conftest import MockAFC, MockLogger, MockPrinter, MockReactor


def _make_canvas_unit(name="CANVAS_1"):
    unit = afcCanvas.__new__(afcCanvas)
    afc = MockAFC()
    afc.error.pause_resume = MagicMock()
    afc.function.is_printing = MagicMock(return_value=False)
    reactor = MockReactor()
    afc.reactor = reactor
    printer = MockPrinter(afc=afc)

    unit.printer = printer
    unit.afc = afc
    unit.function = afc.function
    unit.logger = MockLogger()
    unit.reactor = reactor
    unit.name = name
    unit.type = "canvas"
    unit.prep_distance = 15.0
    unit.prep_speed = 25.0
    unit.short_move_dis = 5.0
    unit.short_moves_speed = 25.0
    unit.short_moves_accel = 400.0
    unit.tool_load_sync_speed_offset = 4.0
    unit.tool_unload_sync_speed_offset = 6.0
    unit.tool_unload_lane_extra_distance = 7.0
    unit.tool_unload_lane_extra_speed = 12.0
    unit.pause_on_tangle = True
    unit._front_cover_state = False
    unit.cutter_sensor_state = False
    unit.enable_9v = None
    unit.enable_24v = None
    unit._tangle_state = False
    return unit


def _make_lane(sensor_states=None):
    lane = MagicMock()
    lane.prep_state = True
    lane.short_move_dis = 4.0
    lane.short_moves_speed = 20.0
    lane.short_moves_accel = 300.0
    lane.led_not_ready = "1,0,0,0"
    lane.led_ready = "0,1,0,0"
    lane.led_loading = "0,0,1,0"
    lane.led_unloading = "1,1,0,0"
    lane.led_tool_loaded = "0,0,1,0"
    lane.led_tool_loaded_idle = "0,0,0,1"
    lane.led_tool_unloaded = "1,0,0,0"
    lane.extruder_obj = MagicMock()
    lane.apply_canvas_led = MagicMock()
    lane.get_speed_accel = MagicMock(return_value=(30.0, 300.0))
    lane.move = MagicMock()
    if sensor_states is None:
        sensor_states = [False]
    lane.get_toolhead_pre_sensor_state = MagicMock(side_effect=sensor_states)
    return lane


def test_prep_load_uses_configured_distance_and_speed():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.move_with_odometer = MagicMock()

    unit.prep_load(lane)

    args, kwargs = lane.move_with_odometer.call_args
    assert args[:2] == (15.0, 25.0)
    assert callable(kwargs["stop_condition"])


def test_prep_post_load_stops_motor():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.disengage_motors = MagicMock()

    unit.prep_post_load(lane)

    lane.apply_canvas_led.assert_called_once_with(unit.afc.led_ready)
    lane.disengage_motors.assert_called_once_with(1.0)
    assert lane._load_state is True


def test_led_callbacks_use_canvas_led_outputs():
    unit = _make_canvas_unit()
    lane = _make_lane()

    unit.lane_tool_loaded(lane)
    unit.lane_tool_unloaded(lane)

    lane.apply_canvas_led.assert_any_call(unit.afc.led_tool_loaded)
    lane.apply_canvas_led.assert_any_call(unit.afc.led_ready)
    lane.extruder_obj.set_status_led.assert_any_call(unit.afc.led_tool_loaded)
    lane.extruder_obj.set_status_led.assert_any_call(unit.afc.led_tool_unloaded)


def test_tangle_callback_pauses_while_printing():
    unit = _make_canvas_unit()
    unit.afc.function.is_printing.return_value = True

    unit.tangle_callback(10.0, True)

    unit.afc.error.AFC_error.assert_called_once_with(
        "CANVAS tangle detected on unit CANVAS_1", pause=True
    )


def test_front_cover_callback_pauses_while_printing():
    unit = _make_canvas_unit()
    unit.afc.function.is_printing.return_value = True

    unit.toolhead_cover_callback(10.0, True)

    unit.afc.error.AFC_error.assert_called_once_with(
        "CANVAS front cover detected on unit CANVAS_1", pause=True
    )


def test_cutter_callback_updates_sensor_state():
    unit = _make_canvas_unit()

    unit.cutter_callback(10.0, True)
    assert unit.cutter_sensor_state is True

    unit.cutter_callback(11.0, False)
    assert unit.cutter_sensor_state is False


def test_lane_unloaded_turns_led_off_and_clears_load_state():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane._load_state = True

    unit.lane_unloaded(lane)

    lane.apply_canvas_led.assert_called_once_with(unit.afc.led_off)
    assert lane._load_state is False


def test_move_to_hub_succeeds_when_sensor_triggers():
    unit = _make_canvas_unit()
    lane = _make_lane(sensor_states=[False, True])

    success, moved, warn = unit.move_to_hub(
        lane, 10.0, MoveDirection.POS, True, speed_mode=SpeedMode.LONG
    )

    assert success is True
    assert moved == 4.0
    assert warn == AFCMoveWarning.NONE


def test_move_to_hub_warns_when_sensor_never_triggers():
    unit = _make_canvas_unit()
    lane = _make_lane(sensor_states=[False, False, False])

    success, moved, warn = unit.move_to_hub(
        lane, 8.0, MoveDirection.POS, True, speed_mode=SpeedMode.LONG
    )

    assert success is False
    assert moved == 8.0
    assert warn == AFCMoveWarning.WARN


def test_system_test_seeds_load_state_from_pressed_prep_endstop():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.prep_state = False
    lane._load_state = False
    lane._afc_prep_done = False
    lane.name = "lane1"
    lane.map = "T0"
    lane.tool_loaded = False
    lane.send_lane_data = MagicMock()
    lane.do_enable = MagicMock()
    lane.set_afc_prep_done = MagicMock()
    lane.endstops = {
        AFCHomingPoints.PREP: {"endstop": MagicMock(query_endstop=MagicMock(return_value=True))}
    }

    unit.system_Test(lane, delay=0.0, assignTcmd=False, enable_movement=False)

    assert lane.prep_state is True
    assert lane._load_state is True
    lane.apply_canvas_led.assert_called_with(unit.afc.led_ready)
    lane.set_afc_prep_done.assert_called_once_with()


def test_system_test_falls_back_to_callback_state_without_prep_endstop():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.prep_state = False
    lane._load_state = True
    lane._afc_prep_done = False
    lane.name = "lane1"
    lane.map = "T0"
    lane.tool_loaded = False
    lane.send_lane_data = MagicMock()
    lane.do_enable = MagicMock()
    lane.set_afc_prep_done = MagicMock()
    lane.endstops = {}

    unit.system_Test(lane, delay=0.0, assignTcmd=False, enable_movement=False)

    assert lane.prep_state is False
    assert lane._load_state is False
    lane.apply_canvas_led.assert_called_with(unit.afc.led_off)