# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from kiwi_scan.epics_wrapper import _CA_LOCK, EpicsPV


class TestEpicsPutCompletion(unittest.TestCase):
    def make_pv(self, result=1):
        wrapper = EpicsPV.__new__(EpicsPV)
        wrapper.pvname = "MOTOR:SET"
        wrapper.timeout = 1.0
        wrapper.queueing_delay = 0.0
        wrapper.last_written = None
        wrapper._pv = Mock()
        wrapper._pv.put.return_value = result
        return wrapper

    def test_submits_async_put_and_accepts_early_callback(self):
        wrapper = self.make_pv()

        def put(value, **kwargs):
            self.assertEqual(value, 3.0)
            self.assertIs(kwargs["wait"], False)
            # pyepics ignores timeout for wait=False; the wait uses put_timeout.
            self.assertNotIn("timeout", kwargs)
            kwargs["callback"](pvname="MOTOR:SET")
            return 1

        wrapper._pv.put.side_effect = put
        completion = wrapper.put_with_completion(3.0, timeout=0.0)

        self.assertTrue(completion.wait())
        self.assertEqual(wrapper.last_written, 3.0)

    def test_submission_returns_before_callback_arrives(self):
        wrapper = self.make_pv()

        completion = wrapper.put_with_completion(3.0, timeout=0.0)

        self.assertFalse(completion.failed)
        with self.assertLogs("kiwi_scan.epics_wrapper", level="WARNING"):
            self.assertFalse(completion.wait())
        wrapper._pv.put.call_args.kwargs["callback"]()
        self.assertTrue(completion.wait())

    def test_wait_and_callback_progress_while_another_thread_holds_ca_lock(self):
        wrapper = self.make_pv()
        completion = wrapper.put_with_completion(3.0, timeout=2.0)
        callback = wrapper._pv.put.call_args.kwargs["callback"]
        finished = threading.Event()
        results = []

        def wait():
            results.append(completion.wait())
            finished.set()

        waiter = threading.Thread(target=wait)
        deliver = threading.Thread(target=callback)
        try:
            with _CA_LOCK, patch("kiwi_scan.epics_wrapper.epics.ca.poll") as poll:
                waiter.start()
                deliver.start()
                self.assertTrue(finished.wait(1.0))
                poll.assert_not_called()
        finally:
            waiter.join(2.0)
            deliver.join(2.0)

        self.assertEqual(results, [True])
        # Neither waiting nor callback delivery makes another PV request.
        self.assertEqual(wrapper._pv.put.call_count, 1)
        wrapper._pv.get.assert_not_called()

    def test_unaccepted_put_is_not_reported_as_failed_and_does_not_wait(self):
        # pyepics returns None when the channel is not connected. Reporting
        # stays as for put(): no failure, but no callback is expected either.
        for result in (None, False, 0, -1):
            with self.subTest(result=result):
                wrapper = self.make_pv(result=result)
                completion = wrapper.put_with_completion(3.0, timeout=60.0)

                self.assertFalse(completion.failed)
                self.assertFalse(completion.pending)
                self.assertFalse(completion.wait())
                self.assertEqual(wrapper.last_written, 3.0)

    def test_submission_exception_returns_a_failed_completion(self):
        wrapper = self.make_pv()
        wrapper._pv.put.side_effect = RuntimeError("disconnected")

        with self.assertLogs("kiwi_scan.epics_wrapper", level="ERROR"):
            completion = wrapper.put_with_completion(3.0, timeout=60.0)

        self.assertTrue(completion.failed)
        self.assertFalse(completion.wait())
        self.assertIsNone(wrapper.last_written)

    def test_waveform_normalization_and_queueing_delay_match_ordinary_put(self):
        wrapper = self.make_pv()
        wrapper.queueing_delay = 0.3

        with patch("kiwi_scan.epics_wrapper.time.sleep") as sleep:
            completion = wrapper.put_with_completion(np.array([1.0, 2.0]), timeout=5.0)

        self.assertFalse(completion.failed)
        self.assertEqual(wrapper._pv.put.call_args.args, ([1.0, 2.0],))
        self.assertEqual(wrapper.last_written, [1.0, 2.0])
        sleep.assert_called_once_with(0.3)

    def test_invalid_completion_timeout_does_not_submit_a_write(self):
        wrapper = self.make_pv()

        with self.assertRaises(ValueError):
            wrapper.put_with_completion(3.0, timeout=-1.0)

        wrapper._pv.put.assert_not_called()

    def test_ordinary_put_retains_existing_interface_and_no_callback(self):
        wrapper = self.make_pv()

        self.assertTrue(wrapper.put(3.0))

        wrapper._pv.put.assert_called_once_with(3.0, timeout=1.0)
        self.assertEqual(wrapper.last_written, 3.0)


if __name__ == "__main__":
    unittest.main()
