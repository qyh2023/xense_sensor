import queue
import unittest

from app import ConnectionSettings, ControllerWorker
from fsc2a_protocol import (
    AXIS_LEAD,
    AXIS_SUBDIVISION,
    COIL_ABSOLUTE,
    COIL_RELATIVE_FORWARD,
    COIL_RELATIVE_REVERSE,
    COIL_SET_POSITION_ZERO,
    COIL_STOP,
    MOVE_DISTANCE,
    MonitorValues,
)


class FakeClient:
    def __init__(self):
        self.calls = []
        self.register_response = [0, 0]
        self.stop_failures_remaining = 0

    def write_int32(self, address, value):
        self.calls.append(("int32", address, value))

    def write_single_coil(self, address, enabled=True):
        self.calls.append(("coil", address, enabled))
        if address == COIL_STOP and self.stop_failures_remaining > 0:
            self.stop_failures_remaining -= 1
            raise TimeoutError("停止应答超时")

    def read_holding_registers(self, address, count, *, slave_id=None):
        self.calls.append(("read", address, count, slave_id))
        return self.register_response[:count]


class AxisConfigClient(FakeClient):
    def __init__(self, lead=10, subdivision=5000):
        super().__init__()
        self.lead = lead
        self.subdivision = subdivision

    def write_int32(self, address, value):
        super().write_int32(address, value)
        if address == AXIS_SUBDIVISION:
            self.subdivision = value

    def read_holding_registers(self, address, count, *, slave_id=None):
        self.calls.append(("read", address, count, slave_id))
        if address == AXIS_LEAD and count == 4:
            return [
                self.lead >> 16,
                self.lead & 0xFFFF,
                self.subdivision >> 16,
                self.subdivision & 0xFFFF,
            ]
        if address == AXIS_SUBDIVISION and count == 2:
            return [self.subdivision >> 16, self.subdivision & 0xFFFF]
        return super().read_holding_registers(address, count, slave_id=slave_id)


class MotionWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.events = queue.Queue()
        settings = ConnectionSettings("COM1", 115200, 1, 0.2)
        self.worker = ControllerWorker(settings, self.events)
        self.worker.pulses_per_mm = 500.0
        self.client = FakeClient()

    def event_kinds(self):
        result = []
        while not self.events.empty():
            result.append(self.events.get_nowait()[0])
        return result

    def test_axis_subdivision_is_synchronised_to_iss35_setting(self):
        client = AxisConfigClient(lead=10, subdivision=5000)

        self.worker._load_and_sync_axis_config(client)

        self.assertIn(("int32", AXIS_SUBDIVISION, 2500), client.calls)
        self.assertEqual(self.worker.subdivision, 2500)
        self.assertEqual(self.worker.pulses_per_mm, 250.0)
        events = []
        while not self.events.empty():
            events.append(self.events.get_nowait())
        self.assertIn(("axis_config_corrected", (5000, 2500)), events)
        self.assertIn(("axis_config", (10, 2500, 250.0)), events)

    def test_matching_axis_subdivision_is_not_rewritten(self):
        client = AxisConfigClient(lead=10, subdivision=2500)

        self.worker._load_and_sync_axis_config(client)

        self.assertNotIn(("int32", AXIS_SUBDIVISION, 2500), client.calls)
        self.assertEqual(self.worker.pulses_per_mm, 250.0)

    def test_calibration_sequence(self):
        self.worker.latest_values = MonitorValues(position=-10_000, speed=0, inputs=0)
        self.worker._start_calibration(self.client)
        self.assertEqual(
            self.client.calls[:3],
            [
                ("coil", COIL_SET_POSITION_ZERO, True),
                ("int32", MOVE_DISTANCE, 100),
                ("coil", COIL_RELATIVE_REVERSE, True),
            ],
        )

        # The controller starts moving; input bits are deliberately unused.
        self.worker._advance_automation(
            self.client, MonitorValues(position=-100, speed=1, inputs=0)
        )

        # The physical limit makes the controller stop. Three stopped polls
        # are treated as a stable controller-side limit stop.
        stopped_at_limit = MonitorValues(position=963, speed=0, inputs=0)
        self.worker._advance_automation(self.client, stopped_at_limit)
        self.worker._advance_automation(self.client, stopped_at_limit)
        self.worker._advance_automation(self.client, stopped_at_limit)
        self.assertIn(("coil", COIL_STOP, True), self.client.calls)

        # The controller position was cleared at the positive limit. Two
        # stopped polls then trigger the 50 mm reverse move.
        limit_zero = MonitorValues(position=0, speed=0, inputs=0)
        self.worker._advance_automation(self.client, limit_zero)
        self.worker._advance_automation(self.client, limit_zero)
        self.assertIn(("int32", MOVE_DISTANCE, 50), self.client.calls)
        self.assertIn(("coil", COIL_RELATIVE_FORWARD, True), self.client.calls)

        # Downward/forward motion starts. With the inverted coordinate and
        # 500 counts/mm, logical 0 mm occurs at raw +25000.
        self.worker._advance_automation(
            self.client, MonitorValues(position=12_500, speed=1, inputs=0)
        )
        stop_count_before_center = self.client.calls.count(("coil", COIL_STOP, True))
        self.worker._advance_automation(
            self.client, MonitorValues(position=25_000, speed=1, inputs=0)
        )
        self.assertEqual(
            self.client.calls.count(("coil", COIL_STOP, True)),
            stop_count_before_center + 1,
        )

        # Two stopped polls at the center mark it as controller zero.
        back_at_center = MonitorValues(position=25_000, speed=0, inputs=0)
        self.worker._advance_automation(self.client, back_at_center)
        self.worker._advance_automation(self.client, back_at_center)
        self.assertIn(("coil", COIL_SET_POSITION_ZERO, True), self.client.calls)
        events = []
        while not self.events.empty():
            events.append(self.events.get_nowait())
        self.assertIn("calibration_done", [kind for kind, _ in events])
        references = [payload for kind, payload in events if kind == "position_reference"]
        self.assertEqual(references, [(0.0, 0.0), (50.0, 50.0), (0.0, 0.0)])
        self.assertIsNone(self.worker.automation)

    def test_home_uses_absolute_target_zero(self):
        self.worker.latest_values = MonitorValues(position=12_500, speed=0, inputs=0)
        self.worker._start_home(self.client)
        self.assertEqual(
            self.client.calls,
            [("int32", MOVE_DISTANCE, 0), ("coil", COIL_ABSOLUTE, True)],
        )

    def test_position_counts_are_not_treated_as_millimetres(self):
        self.worker.latest_values = MonitorValues(position=0, speed=0, inputs=0)
        self.worker._start_calibration(self.client)
        self.worker._advance_automation(
            self.client, MonitorValues(position=-124, speed=1, inputs=0)
        )
        self.assertIsNotNone(self.worker.automation)
        self.assertEqual(self.worker.automation["phase"], "seek_limit")
        self.assertNotIn("operation_error", self.event_kinds())

    def test_force_limit_stops_calibration_immediately(self):
        self.worker.force_limit = 10.0
        self.worker.force_decimal_places = 2
        self.worker.force_transmitter_slave = 2
        self.worker.automation = {"kind": "calibration"}
        self.client.register_response = [0x0000, 0x04D2]  # 12.34

        self.worker._poll_transmitter_force(self.client)

        self.assertIn(("coil", COIL_STOP, True), self.client.calls)
        self.assertIsNone(self.worker.automation)
        self.assertTrue(self.worker.force_limit_tripped)
        events = []
        while not self.events.empty():
            events.append(self.events.get_nowait())
        trips = [payload for kind, payload in events if kind == "force_limit_tripped"]
        self.assertEqual(trips, [(12.34, 10.0, "calibration", None)])

    def test_force_limit_uses_absolute_force(self):
        self.worker.force_limit = 10.0
        self.worker.force_decimal_places = 2
        self.worker.force_transmitter_slave = 2
        self.client.register_response = [0xFFFF, 0xFB2E]  # -12.34

        self.worker._poll_transmitter_force(self.client)

        self.assertIn(("coil", COIL_STOP, True), self.client.calls)
        self.assertTrue(self.worker.force_limit_tripped)

    def test_force_limit_latches_before_stop_reply_and_retries(self):
        self.worker.force_limit = 10.0
        self.worker.force_decimal_places = 2
        self.worker.force_transmitter_slave = 2
        self.worker.automation = {"kind": "calibration"}
        self.client.register_response = [0x0000, 0x04D2]  # 12.34
        self.client.stop_failures_remaining = 3

        tripped = self.worker._poll_transmitter_force(self.client)

        self.assertTrue(tripped)
        self.assertIsNone(self.worker.automation)
        self.assertTrue(self.worker.force_limit_tripped)
        self.assertEqual(
            self.client.calls.count(("coil", COIL_STOP, True)),
            3,
        )
        events = []
        while not self.events.empty():
            events.append(self.events.get_nowait())
        trips = [payload for kind, payload in events if kind == "force_limit_tripped"]
        self.assertEqual(
            trips,
            [(12.34, 10.0, "calibration", "停止应答超时")],
        )

    def test_manual_and_force_stop_share_emergency_path(self):
        self.worker.automation = {"kind": "home"}

        interrupted_kind, stop_error = self.worker._emergency_stop(self.client)

        self.assertEqual(interrupted_kind, "home")
        self.assertIsNone(stop_error)
        self.assertIsNone(self.worker.automation)
        self.assertIn(("coil", COIL_STOP, True), self.client.calls)


if __name__ == "__main__":
    unittest.main()
