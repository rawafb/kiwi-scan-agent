# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

import threading
import unittest
from unittest.mock import patch

from kiwi_scan.epics_wrapper import PutCompletion


class TestPutCompletion(unittest.TestCase):
    def test_callback_before_wait_is_retained(self):
        completion = PutCompletion("MOTOR:SET", 0.0)

        completion.callback(pvname="MOTOR:SET", extra="ignored")

        self.assertTrue(completion.wait())
        self.assertFalse(completion.failed)

    def test_callback_releases_a_wait_on_another_thread(self):
        completion = PutCompletion("MOTOR:SET", 2.0)
        started = threading.Event()
        finished = threading.Event()
        results = []

        def wait():
            started.set()
            results.append(completion.wait())
            finished.set()

        worker = threading.Thread(target=wait)
        worker.start()
        try:
            self.assertTrue(started.wait(1.0))
            self.assertFalse(finished.wait(0.01))
            completion.callback()
            self.assertTrue(finished.wait(1.0))
        finally:
            completion.callback()
            worker.join(1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [True])

    def test_late_callback_cannot_complete_a_subsequent_put(self):
        previous = PutCompletion("MOTOR:SET", 0.0)
        current = PutCompletion("MOTOR:SET", 0.0)

        with self.assertLogs("kiwi_scan.epics_wrapper", level="WARNING"):
            self.assertFalse(previous.wait())
        previous.callback()

        with self.assertLogs("kiwi_scan.epics_wrapper", level="WARNING"):
            self.assertFalse(current.wait())
        current.callback()
        self.assertTrue(current.wait())

    def test_timeout_warns_and_returns_false(self):
        completion = PutCompletion("MOTOR:SET", 0.0)

        with self.assertLogs("kiwi_scan.epics_wrapper", level="WARNING") as captured:
            self.assertFalse(completion.wait())

        self.assertIn("MOTOR:SET", captured.output[0])

    def test_failed_submission_needs_no_callback_or_timeout_wait(self):
        completion = PutCompletion("MOTOR:SET", 60.0)
        completion.fail()

        self.assertTrue(completion.failed)
        self.assertFalse(completion.wait())
        # A late callback cannot turn a failed submission into success.
        completion.callback()
        self.assertFalse(completion.wait())

    def test_stop_request_takes_priority_over_already_received_callback(self):
        completion = PutCompletion("MOTOR:SET", 60.0)
        completion.callback()
        stop = threading.Event()
        stop.set()

        self.assertFalse(completion.wait(stop_event=stop))

    def test_stop_request_interrupts_an_active_wait(self):
        completion = PutCompletion("MOTOR:SET", 60.0)
        stop = threading.Event()
        started = threading.Event()
        results = []

        def wait():
            started.set()
            results.append(completion.wait(stop_event=stop))

        worker = threading.Thread(target=wait)
        worker.start()
        try:
            self.assertTrue(started.wait(1.0))
            stop.set()
            worker.join(1.0)
            self.assertFalse(worker.is_alive())
        finally:
            completion.callback()
            worker.join(1.0)

        self.assertEqual(results, [False])

    def test_pending_until_callback_or_deadline(self):
        completion = PutCompletion("MOTOR:SET", 60.0)
        self.assertTrue(completion.pending)
        completion.callback()
        self.assertFalse(completion.pending)
        self.assertTrue(completion.completed)

        expired = PutCompletion("MOTOR:SET", 0.0)
        self.assertFalse(expired.pending)

    def test_deadline_counts_from_submission_not_from_wait(self):
        with patch("kiwi_scan.epics_wrapper.time.monotonic", side_effect=[100.0, 160.0]):
            completion = PutCompletion("MOTOR:SET", 50.0)
            with self.assertLogs("kiwi_scan.epics_wrapper", level="WARNING"):
                self.assertFalse(completion.wait())

    def test_cancel_releases_an_active_wait_and_clears_pending(self):
        completion = PutCompletion("MOTOR:SET", 60.0)
        started = threading.Event()
        results = []

        def wait():
            started.set()
            results.append(completion.wait())

        worker = threading.Thread(target=wait)
        worker.start()
        try:
            self.assertTrue(started.wait(1.0))
            completion.cancel()
            worker.join(1.0)
            self.assertFalse(worker.is_alive())
        finally:
            completion.callback()
            worker.join(1.0)

        self.assertEqual(results, [False])
        self.assertTrue(completion.cancelled)
        self.assertFalse(completion.pending)
        self.assertFalse(completion.completed)

    def test_unarmed_put_is_not_failed_and_does_not_wait(self):
        completion = PutCompletion("MOTOR:SET", 60.0)
        completion.mark_unarmed()

        self.assertFalse(completion.failed)
        self.assertFalse(completion.pending)
        self.assertFalse(completion.wait())

    def test_timeout_must_be_finite_and_nonnegative(self):
        for timeout in (-1.0, float("inf"), float("-inf"), float("nan")):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                PutCompletion("MOTOR:SET", timeout)


if __name__ == "__main__":
    unittest.main()
