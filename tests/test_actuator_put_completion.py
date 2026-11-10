# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

import threading
import unittest
from unittest.mock import Mock, patch

from kiwi_scan.actuator.multi import MultiActuator
from kiwi_scan.actuator_concrete.single_epics import EpicsActuator
from kiwi_scan.datamodels import ActuatorConfig, ConfigError, JogConfig
from kiwi_scan.epics_wrapper import PutCompletion
from kiwi_scan.test_support import make_fake_actuator_pv_class

MODULE = "kiwi_scan.actuator_concrete.single_epics"


class _CompletionFactory:
    """Stand-in for EpicsPV.put_with_completion that records the requests."""

    def __init__(self, timeout_override=None, complete=False):
        self.calls = []
        self.completions = []
        self.timeout_override = timeout_override
        self.complete = complete

    def __call__(self, value, *, timeout):
        self.calls.append((value, timeout))
        t = timeout if self.timeout_override is None else self.timeout_override
        completion = PutCompletion("MOTOR:SET", t)
        if self.complete:
            completion.callback()
        self.completions.append(completion)
        return completion


class TestActuatorPutCompletion(unittest.TestCase):
    def setUp(self):
        fake_pv_class = make_fake_actuator_pv_class()
        patcher = patch(f"{MODULE}.EpicsPV", fake_pv_class)
        patcher.start()
        self.addCleanup(patcher.stop)

    def make_actuator(self, factory=None, **overrides):
        values = {
            "pv": "MOTOR:SET",
            "put_timeout": 5.0,
            "queueing_delay": 0.0,
            "dwell_time": 0.0,
        }
        values.update(overrides)
        actuator = EpicsActuator(ActuatorConfig(**values))
        if actuator.pv is not None:
            actuator.pv.put_with_completion = factory or _CompletionFactory()
        return actuator

    # --- issuing moves -------------------------------------------------

    def test_move_arms_callback_without_waiting(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(factory)

        actuator.move(7.0)

        self.assertEqual(factory.calls, [(7.0, 5.0)])
        self.assertIs(actuator._pending_put, factory.completions[0])
        self.assertTrue(actuator._pending_put.pending)
        self.assertEqual(actuator.pv.put_calls, [])

    def test_disabled_feature_keeps_plain_put_and_start_pv(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(factory, put_timeout=0.0, start_pv="MOTOR:START")

        with patch.object(actuator, "wait_until_done") as done:
            actuator.run_move(7.0, sync=True)

        self.assertEqual(factory.calls, [])
        self.assertEqual(actuator.pv.put_calls, [7.0])
        self.assertEqual(actuator.start_pv.put_calls, [0.0])
        self.assertIsNone(actuator._pending_put)
        done.assert_called_once_with(7.0)

    def test_start_pv_keeps_put_completion_enabled(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(
            factory, start_pv="MOTOR:START", start_command=2.0,
            status_pv="MOTOR:DMOV", ready_value=1,
        )
        actuator.status_pv._value = 1
        actuator.start_pv.put_with_completion = Mock()

        actuator.move(7.0)

        self.assertEqual(actuator.put_timeout, 5.0)
        self.assertEqual(actuator.config.put_timeout, 5.0)
        self.assertEqual(factory.calls, [(7.0, 5.0)])
        self.assertEqual(actuator.pv.put_calls, [])
        self.assertEqual(actuator.start_pv.put_calls, [2.0])
        actuator.start_pv.put_with_completion.assert_not_called()
        self.assertIs(actuator._pending_put, factory.completions[0])
        self.assertTrue(actuator._pending_put.pending)
        self.assertFalse(actuator.is_ready())
        self.assertTrue(actuator.is_moving())

        factory.completions[0].callback()
        self.assertTrue(actuator.is_ready())
        self.assertFalse(actuator.is_moving())

    def test_sync_move_sends_start_before_waiting_for_put_completion(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(
            factory, start_pv="MOTOR:START", start_command=2.0,
        )
        trace = []
        original_wait = PutCompletion.wait

        def put(value, *, timeout):
            trace.append(("setpoint", value))
            return factory(value, timeout=timeout)

        def start(value):
            trace.append(("start", value))
            self.assertTrue(factory.completions[0].pending)
            factory.completions[0].callback()
            return True

        def wait(completion, stop_event=None):
            trace.append(("wait",))
            return original_wait(completion, stop_event=stop_event)

        actuator.pv.put_with_completion = put
        actuator.start_pv.put = Mock(side_effect=start)
        actuator.start_pv.put_with_completion = Mock()

        with patch.object(PutCompletion, "wait", wait):
            actuator.run_move(7.0, sync=True)

        self.assertEqual(trace, [("setpoint", 7.0), ("start", 2.0), ("wait",)])
        actuator.start_pv.put.assert_called_once_with(2.0)
        actuator.start_pv.put_with_completion.assert_not_called()
        self.assertTrue(factory.completions[0].completed)

    def test_sync_move_waits_for_completion_before_motion_checks(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(factory, queueing_delay=0.2)
        trace = []

        def put(value, *, timeout):
            trace.append(("setpoint", value))
            return factory(value, timeout=timeout)

        actuator.pv.put_with_completion = put

        def fake_wait(_self, stop_event=None):
            trace.append(("completion", stop_event))
            return True

        with patch.object(PutCompletion, "wait", fake_wait), patch.object(
            actuator, "in_position_check", side_effect=lambda *a, **k: trace.append(("band",)) or True
        ), patch(f"{MODULE}.time.sleep", side_effect=lambda delay: trace.append(("delay", delay))):
            actuator.in_pos_band = 0.1
            actuator.run_move(7.0, sync=True)

        self.assertEqual(
            trace,
            [("setpoint", 7.0), ("delay", 0.2), ("completion", None), ("band",)],
        )

    def test_completed_put_skips_startup_wait_but_checks_ready(self):
        actuator = self.make_actuator(_CompletionFactory(complete=True), status_pv="MOTOR:DMOV", ready_value=1)
        actuator.status_pv._value = 1

        with patch.object(actuator, "wait_for_startup") as startup, patch.object(
            actuator, "_wait_for_condition", return_value=True
        ) as cond:
            actuator.run_move(7.0, sync=True)

        startup.assert_not_called()
        cond.assert_called_once()
        self.assertEqual(cond.call_args.args[0], actuator.is_ready)

    def test_timed_out_put_keeps_startup_and_ready_checks(self):
        actuator = self.make_actuator(
            _CompletionFactory(timeout_override=0.0), status_pv="MOTOR:DMOV"
        )

        with self.assertLogs("kiwi_scan.epics_wrapper", level="WARNING"), patch.object(
            actuator, "wait_for_startup_and_done"
        ) as status_wait:
            actuator.run_move(7.0, sync=True)

        status_wait.assert_called_once()

    def test_failed_submission_keeps_existing_motion_checks_without_waiting(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(factory, in_position_band=0.1)

        def failing(value, *, timeout):
            completion = factory(value, timeout=timeout)
            completion.fail()
            return completion

        actuator.pv.put_with_completion = failing

        with self.assertLogs(MODULE, level="ERROR"), patch.object(
            actuator, "in_position_check", return_value=True
        ) as band:
            actuator.run_move(7.0, sync=True)

        band.assert_called_once()
        self.assertFalse(actuator.is_moving())

    def test_sync_relative_fallback_tracks_main_setpoint_and_computed_target(self):
        factory = _CompletionFactory(complete=True)
        actuator = self.make_actuator(factory, rb_pv="MOTOR:RBV")
        actuator.rb_pv._value = 4.0

        with patch.object(actuator, "in_position_check") as band:
            actuator.run_rel_move(2.0, sync=True)

        self.assertEqual(factory.calls, [(6.0, 5.0)])
        band.assert_not_called()

    def test_relative_pv_is_a_plain_put_and_forgets_older_completion(self):
        factory = _CompletionFactory(complete=True)
        actuator = self.make_actuator(factory, rel_pv="MOTOR:REL")
        actuator.move(1.0)
        self.assertIsNotNone(actuator._pending_put)

        with patch.object(actuator, "wait_for_startup_and_done"), patch.object(actuator, "dwell"):
            actuator.run_rel_move(2.0, sync=True)

        self.assertEqual(factory.calls, [(1.0, 5.0)])
        self.assertEqual(actuator.rel_pv.put_calls, [2.0])
        self.assertIsNone(actuator._pending_put)

    def test_other_setters_and_jog_remain_ordinary_writes(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(
            factory,
            rb_pv="MOTOR:RBV",
            cmd_pv="MOTOR:CMD",
            cmdvel_pv="MOTOR:CMDVELO",
            velocity_pv="MOTOR:VELO",
            stop_pv="MOTOR:STOP",
            jog=JogConfig(velocity_pv="MOTOR:JOGVELO", command_pv="MOTOR:JOG", command_pos=1),
        )
        other_pvs = (
            actuator.rb_pv, actuator.cmd_pv, actuator.cmdvel_pv,
            actuator.velocity_pv, actuator.stop_pv,
            actuator.jog_velocity_pv, actuator.jog_command_pv,
        )
        for pv in other_pvs:
            pv.put_with_completion = Mock()

        actuator.rbv = 2.0
        actuator.cmdv = 3.0
        actuator.cmdvelv = 4.0
        actuator.set_velocity(5.0)
        actuator.stop()
        with patch.object(actuator, "wait_for_startup_and_done"):
            actuator.jog(1.0, sync=True)

        self.assertEqual(factory.calls, [])
        for pv in other_pvs:
            with self.subTest(pv=pv.pvname):
                pv.put_with_completion.assert_not_called()
                self.assertTrue(pv.put_calls)

    # --- status ----------------------------------------------------------

    def test_pending_put_reports_busy_without_status_pv(self):
        factory = _CompletionFactory()
        actuator = self.make_actuator(factory)
        self.assertTrue(actuator.is_ready())

        actuator.move(7.0)
        self.assertFalse(actuator.is_ready())
        self.assertTrue(actuator.is_moving())
        self.assertTrue(actuator.wait_for_startup())

        factory.completions[0].callback()
        self.assertTrue(actuator.is_ready())
        self.assertFalse(actuator.is_moving())

    def test_pending_put_overrides_ready_status_pv(self):
        actuator = self.make_actuator(status_pv="MOTOR:DMOV", ready_value=1)
        actuator.status_pv._value = 1
        actuator.move(7.0)

        self.assertFalse(actuator.is_ready())

    def test_expired_put_no_longer_reports_busy(self):
        actuator = self.make_actuator(_CompletionFactory(timeout_override=0.0))
        actuator.move(7.0)

        self.assertTrue(actuator.is_ready())

    # --- stopping ------------------------------------------------------

    def test_stop_clears_busy_state_and_releases_waiter(self):
        actuator = self.make_actuator(stop_pv="MOTOR:STOP")
        actuator.move(7.0)
        finished = threading.Event()

        def wait():
            actuator.wait_until_done(7.0)
            finished.set()

        worker = threading.Thread(target=wait)
        worker.start()
        try:
            self.assertFalse(finished.wait(0.1))
            actuator.stop()
            self.assertTrue(finished.wait(1.0))
        finally:
            actuator._pending_put.cancel()
            worker.join(1.0)

        self.assertTrue(actuator.is_ready())
        self.assertEqual(actuator.stop_pv.put_calls, [0.0])

    def test_stop_event_interrupts_put_wait_in_wait_until_done(self):
        actuator = self.make_actuator(in_position_band=0.1)
        actuator.move(7.0)
        stop = threading.Event()
        finished = threading.Event()

        def wait():
            actuator.wait_until_done(7.0, stop_event=stop)
            finished.set()

        with patch.object(actuator, "in_position_check") as band:
            worker = threading.Thread(target=wait)
            worker.start()
            try:
                self.assertFalse(finished.wait(0.1))
                stop.set()
                self.assertTrue(finished.wait(1.0))
            finally:
                actuator._pending_put.cancel()
                worker.join(1.0)

        band.assert_not_called()
        # A stop request on the scan side leaves the IOC busy state untouched.
        actuator._pending_put = None

    def test_close_cancels_pending_put(self):
        actuator = self.make_actuator()
        actuator.move(7.0)
        pending = actuator._pending_put

        actuator.close()

        self.assertTrue(pending.cancelled)

    # --- multi-axis ----------------------------------------------------

    def test_multi_axis_issues_all_puts_then_waits_for_all_callbacks(self):
        first_factory = _CompletionFactory()
        second_factory = _CompletionFactory()
        first = self.make_actuator(first_factory, pv="FIRST:SET")
        second = self.make_actuator(second_factory, pv="SECOND:SET")
        group = MultiActuator([first, second], ActuatorConfig())

        def deliver():
            # Both requests are submitted before any callback is delivered.
            first_factory.completions[0].callback()
            second_factory.completions[0].callback()

        timer = threading.Timer(0.05, deliver)
        timer.start()
        try:
            group.run_move([1.0, 2.0], sync=True)
        finally:
            timer.cancel()

        self.assertEqual(first_factory.calls, [(1.0, 5.0)])
        self.assertEqual(second_factory.calls, [(2.0, 5.0)])
        self.assertTrue(first_factory.completions[0].completed)
        self.assertTrue(second_factory.completions[0].completed)


class TestPutCompletionConfiguration(unittest.TestCase):
    def test_feature_defaults_to_disabled(self):
        self.assertEqual(ActuatorConfig().put_timeout, 0.0)

    def test_configuration_parser_retains_put_timeout(self):
        config = ActuatorConfig.from_dict({"pv": "MOTOR:SET", "put_timeout": 12.0})
        self.assertEqual(config.put_timeout, 12.0)

    def test_configuration_accepts_numeric_string_and_int(self):
        self.assertEqual(ActuatorConfig(put_timeout="12.0").put_timeout, 12.0)
        self.assertEqual(ActuatorConfig(put_timeout=3).put_timeout, 3.0)

    def test_configuration_accepts_put_timeout_with_start_pv(self):
        config = ActuatorConfig.from_dict({"pv": "M:SET", "start_pv": "M:GO", "put_timeout": 1.0})
        self.assertEqual(config.put_timeout, 1.0)

    def test_configuration_rejects_invalid_timeouts(self):
        for timeout in (-1.0, float("inf"), float("nan"), None, "invalid", True, False):
            with self.subTest(timeout=timeout), self.assertRaises(ConfigError) as ctx:
                ActuatorConfig(put_timeout=timeout)
            self.assertIn("put_timeout", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
