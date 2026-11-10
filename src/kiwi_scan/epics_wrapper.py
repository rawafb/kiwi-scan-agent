# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Dict, Optional

import epics

logger = logging.getLogger(__name__)
_CA_LOCK = threading.Lock()


class PutCompletion:
    """
    Tracks one CA put-callback request (``ca_array_put_callback``).

    The IOC sends the callback after the record, including asynchronous
    device support and forward links, has finished processing the write.
    The timeout runs from the submission (creation of this object), so
    ``pending`` and ``wait()`` share the same deadline.

    States:
      - completed: the callback arrived.
      - failed:    submission raised; no callback will arrive.
      - unarmed:   CA did not accept the request (e.g. not connected);
                   no callback will arrive, but the put is not reported
                   as failed (same reporting as ``EpicsPV.put()``).
      - cancelled: ``cancel()`` was called, e.g. by ``Actuator.stop()``.
    """

    def __init__(self, pvname: str, timeout: float):
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError(f"put completion timeout must be finite and non-negative, got {timeout!r}")
        self.pvname = pvname
        self.timeout = timeout
        self._deadline = time.monotonic() + timeout
        self._event = threading.Event()
        self._completed = False
        self._failed = False
        self._unarmed = False
        self._cancelled = False

    @property
    def failed(self) -> bool:
        """True if the submission raised; no callback will arrive."""
        return self._failed

    @property
    def completed(self) -> bool:
        """True if the IOC reported completion (and nothing overrode it)."""
        return self._completed and not (self._failed or self._unarmed or self._cancelled)

    @property
    def cancelled(self) -> bool:
        return self._cancelled

    @property
    def pending(self) -> bool:
        """True while a callback is still expected and the deadline has not passed."""
        if self._event.is_set():
            return False
        return time.monotonic() < self._deadline

    def callback(self, **_kwargs: Any) -> None:
        """CA-thread callback: signal completion."""
        self._completed = True
        self._event.set()

    def fail(self) -> None:
        """Set submission failure and release any waiter."""
        self._failed = True
        self._event.set()

    def mark_unarmed(self) -> None:
        """No callback will arrive, but do not report a failed put."""
        self._unarmed = True
        self._event.set()

    def cancel(self) -> None:
        """Stop waiting and stop reporting this put as pending. Safe from any thread."""
        self._cancelled = True
        self._event.set()

    def wait(self, stop_event: Optional[threading.Event] = None) -> bool:
        """
        Wait for completion until the deadline.

        Returns:
            - True on completion
            - False on failure, missing callback, cancellation, stop request or timeout
        """
        logger.debug("Waiting for put completion on %s (timeout %.3fs)", self.pvname, self.timeout)
        while True:
            if stop_event is not None and stop_event.is_set():
                logger.debug("Put completion wait stopped for %s", self.pvname)
                return False
            if self._cancelled:
                logger.debug("Put completion cancelled for %s", self.pvname)
                return False
            if self._failed or self._unarmed:
                logger.debug("No put completion expected for %s", self.pvname)
                return False
            if self._completed:
                logger.debug("Put completed on %s", self.pvname)
                return True
            remaining = self._deadline - time.monotonic()
            if remaining <= 0:
                logger.warning("Put completion for %s not reached within %.3fs", self.pvname, self.timeout)
                return False
            # Short slices keep stop_event responsive without a second event.
            self._event.wait(min(0.05, remaining))


def ensure_ca_context() -> None:
    """ Attach the current thread to the initial EPICS CA context if possible.  """
    try:
        epics.ca.use_initial_context()
    except epics.ca.ChannelAccessException:
        logger.debug("Could not attach initial EPICS CA context", exc_info=True)

def _safe_poll() -> None:
    """ 
    try: Yield to EPICS CA
    Best-effort; varies across pyepics versions.
    """
    try:
        time.sleep(0.01)
        epics.ca.poll()
    except Exception:
        logger.debug("EPICS CA poll failed", exc_info=True)

class EpicsPV:
    """
    Safe EPICS PV wrapper.
    Some CA-touching calls are serialized with _CA_LOCK to avoid segfaults from EPICS base.
    """
    def __init__(
        self,
        pvname: str,
        timeout: float = 1.0,
        queueing_delay: float = 0.01,
        auto_monitor: bool = False,
        callback: Optional[Callable[..., None]] = None,
        connection_timeout: Optional[float] = None,
    ):
        if queueing_delay < 0:
            raise ValueError(f"queueing_delay must be non-negative, got {queueing_delay}")

        self.pvname = pvname
        self.timeout = float(timeout)
        self.queueing_delay = float(queueing_delay)
        self.connection_timeout = float(connection_timeout) if connection_timeout is not None else max(10.0, self.timeout)

        self._pv: Optional[epics.PV] = None
        self._callback_refs: Dict[int, Callable[..., None]] = {}
        self.last_written: Any = None

        logger.debug("Creating PV %s", pvname)

        # Create + connect (serialized)
        def _create_and_connect() -> bool:
            try:
                pv = epics.PV(pvname, auto_monitor=False, connection_timeout=self.connection_timeout)
            except TypeError:
                pv = epics.PV(pvname, auto_monitor=False)

            self._pv = pv
            logger.debug("Wait for connection to %s", pvname)
            ok = pv.wait_for_connection(timeout=self.connection_timeout)
            _safe_poll()
            return ok

        try:
            ok = self._ca(_create_and_connect)
            if not ok:
                raise ConnectionError(f"EPICS PV '{pvname}' did not connect within {self.connection_timeout:.2f}s")
        except BaseException:
            try:
                self.close()
            except Exception:
                logger.debug("Failed to close PV %s after connection failed", pvname, exc_info=True)
            raise

        # Enable monitoring AFTER connection (serialized)
        if auto_monitor:
            self._ca(self._enable_monitoring)

        # Optional callback
        if callback and auto_monitor:
            self.add_callback(callback, run_now=False)

    # ----------------- internal helpers -----------------

    def _ca(self, fn: Callable[[], Any]) -> Any:
        """Run a function while holding the global CA lock."""
        with _CA_LOCK:
            return fn()

    def _require_pv(self) -> epics.PV:
        if self._pv is None:
            raise RuntimeError(f"PV '{self.pvname}' not initialized")
        return self._pv

    def _enable_monitoring(self) -> None:
        pv = self._require_pv()
        try:
            pv.auto_monitor = True
        except Exception:
            logger.debug("Failed to enable EPICS auto_monitor", exc_info=True)
        _safe_poll()

    @property
    def nelm(self) -> Optional[int]:
        """Return the configured number of PV elements when available."""
        pv = self._require_pv()
        raw_nelm = getattr(pv, "nelm", None)
        if raw_nelm is None:
            return None
        try:
            return int(raw_nelm)
        except (TypeError, ValueError):
            logger.debug(
                "Invalid NELM value for PV %s: %r",
                self.pvname,
                raw_nelm,
            )
            return None

    def _wrap_callback(self, user_cb: Callable[..., None]) -> Callable[..., None]:
        def _cb(pvname=None, value=None, **kwargs):
            try:
                user_cb(pvname=pvname, value=value, **kwargs)
            except TypeError:
                # logger.error("Callback mismatch: %s %s", user_cb, e)
                user_cb(pvname, value)
            except Exception:
                logger.exception("Error in PV callback for '%s'", self.pvname)
        return _cb

    # ----------------- public API -----------------

    def get(self, *, use_monitor: bool = False, timeout: Optional[float] = None) -> Any:
        """ Get PV and floor timeout """
        pv = self._require_pv()
        # TODO: change to simply self.timeout if timeout is None?
        t = min(self.timeout, 1.0) if timeout is None else float(timeout)
        return pv.get(timeout=t, use_monitor=use_monitor)
    
    def get_fast(self) -> Optional[Dict[str, Any]]:
        if self._pv is None:
            return None
        return self._pv.get_with_metadata(use_monitor=True)

    def get_with_metadata(self, *, use_monitor: bool = False, timeout: Optional[float] = None, full: Optional[bool] = False) -> Optional[Dict[str, Any]]:
        """ Return dict with value + timestamp-ish metadata. TODO: how about full meta data? """
        pv = self._require_pv()
        t = min(self.timeout, 0.2) if timeout is None else float(timeout)

        def _do() -> Optional[Dict[str, Any]]:
            if full:
                meta = pv.get_with_metadata(use_monitor=use_monitor, timeout=t)
                return meta
            else:
                """ Thread safe usage: self._ca(_do) """
                val = pv.get(timeout=t, use_monitor=use_monitor)
                if val is None:
                    return None
                # pyepics has pv.timestamp, pv.severity, pv.status sometimes
                timestamp = pv.timestamp
                meta: Dict[str, Any] = {"value": val}
                meta["pvname"] = self.pvname
                if timestamp is None:
                    logger.debug("No timestamp available for PV %s; using local time", self.pvname)
                    meta["timestamp"] = time.time()
                else:
                    try:
                        meta["timestamp"] = float(timestamp)
                    except (TypeError, ValueError, OverflowError):
                        logger.debug("Invalid timestamp for PV %s; using local time", self.pvname, exc_info=True)
                        meta["timestamp"] = time.time()
                return meta

        return _do()

    def put(self, value: Any) -> bool:
        """ Put a value to the PV.
        Supports scalar writes and waveform (array) writes.
        """
        pv = self._require_pv()

        # Normalization (e.g. numpy arrays)
        v = value
        tolist = getattr(v, "tolist", None)
        if callable(tolist):
            v = tolist()
        try:
            pv.put(v, timeout=self.timeout)
            if self.queueing_delay > 0:
                time.sleep(self.queueing_delay)
            logger.debug("Set PV %s = %r", self.pvname, v)
            self.last_written = v
            return True
        except Exception as e: # noqa BLE001
            logger.error("Failed to set PV %s to %r: %s", self.pvname, v, e)
            return False

    def put_with_completion(self, value: Any, *, timeout: float) -> PutCompletion:
        """
        Submit a non-blocking put with a CA completion callback.

        Returns a PutCompletion that reports when the IOC has finished processing the write. 
        Failure reporting matches ``put()``: only an exception counts as a failed put.

        TODO: a callback that never arrives (e.g. after a timeout or a cancel) 
              stays registered in pyepics (``ca._put_completes``) until the channel is cleared.
        """
        completion = PutCompletion(self.pvname, timeout)
        pv = self._require_pv()
        v = value
        tolist = getattr(v, "tolist", None)
        if callable(tolist):
            v = tolist()
        try:
            result = pv.put(v, wait=False, callback=completion.callback)
            # pyepics returns None when the channel is not connected; 
            # CA errors: do not report this as a failed write, but do not wait for a callback that cannot arrive.
            # TODO: pass errors here?
            if result is None or result <= 0:
                completion.mark_unarmed()
                logger.debug("Put to %s = %r has no completion callback (status %r)",
                             self.pvname, v, result)
            else:
                logger.debug("Submitted put to %s = %r with completion callback", self.pvname, v)
            self.last_written = v
            if self.queueing_delay > 0:
                time.sleep(self.queueing_delay)
        except Exception as exc:  # noqa: BLE001 - match ordinary put failure handling
            completion.fail()
            logger.error("Failed to set PV %s to %r with completion: %s", self.pvname, v, exc)
        return completion

    def add_callback(self, callback: Callable[..., None], **kwargs: Any) -> int:
        pv = self._require_pv()
        wrapped = self._wrap_callback(callback)

        def _do_add() -> int:
            # Ensure monitoring (best effort)
            try:
                pv.auto_monitor = True
            except Exception:
                logger.debug("Failed to enable auto_monitor for PV %s", self.pvname, exc_info=True)
            _safe_poll()
            try:
                cb_index = pv.add_callback(wrapped, **kwargs)
            except TypeError:
                # Some versions don’t accept run_now, etc.
                kwargs.pop("run_now", None)
                kwargs.pop("with_ctrlvars", None)
                cb_index = pv.add_callback(wrapped, **kwargs)
            _safe_poll()

            if cb_index is None:
                raise RuntimeError(
                    f"Adding callback to PV {self.pvname!r} returned no callback index"
                )
            self._callback_refs[cb_index] = wrapped
            return cb_index

        return self._ca(_do_add)

    def remove_callback(self, index: int) -> None:
        """Remove one callback by the index returned from add_callback()."""
        pv = self._require_pv()

        def _do_remove() -> None:
            pv.remove_callback(index)
            self._callback_refs.pop(index, None)

        self._ca(_do_remove)

    def disconnect(self) -> None:
        """Disconnect this PV from Channel Access."""
        pv = self._require_pv()
        self._ca(pv.disconnect)

    def close(self) -> None:
        """ Release PV resources. Can be called repeatedly from users after workes have stopped.  """
        with _CA_LOCK:
            pv = self._pv
            if pv is None:
                return
            try:
                pv.clear_callbacks()
            except Exception:
                logger.debug("Failed to clear callbacks while closing PV %s", self.pvname, exc_info=True)
            # Keep PV if disconnect fails
            pv.disconnect()
            self._callback_refs.clear()
            self._pv = None

    def clear_callbacks(self) -> None:
        pv = self._require_pv()

        def _do_clear() -> None:
            try:
                pv.clear_callbacks()
            except Exception:
                logger.debug("Failed to clear callbacks for PV %s", self.pvname, exc_info=True)
            self._callback_refs.clear()
            _safe_poll()

        self._ca(_do_clear)

    def check_pv(self) -> None:
        if not self._pv:
            raise RuntimeError(f"PV '{self.pvname}' not initialized")
        ok = self._ca(lambda: bool(getattr(self._pv, "connected", False)))
        if not ok:
            raise ConnectionError(f"PV '{self.pvname}' not connected")
    
    # Ignore strict connection handling above
    @classmethod
    def create_monitor(cls, pvname: str, **kwargs) -> EpicsPV:
        """
        Create a PV for monitoring without blocking on connection.
        """
        obj = cls.__new__(cls)  # bypass __init__

        obj.pvname = pvname
        obj.timeout = float(kwargs.get("timeout", 1.0))
        obj.queueing_delay = float(kwargs.get("queueing_delay", 0.01))
        obj.connection_timeout = float(kwargs.get("connection_timeout", 10.0))
        obj._callback_refs = {}
        obj.last_written = None

        # create PV WITHOUT waiting
        obj._pv = epics.PV(pvname, auto_monitor=True)

        return obj

NO_ALARM = 0
MINOR_ALARM = 1
MAJOR_ALARM = 2
INVALID_ALARM = 3

STATUS_NAMES = {
    0: "NO_ALARM",
    1: "READ",
    2: "WRITE",
    3: "HIHI",
    4: "HIGH",
    5: "LOLO",
    6: "LOW",
    7: "STATE",
    8: "COS",
    9: "COMM",
    10: "TIMEOUT",
    11: "HWLIMIT",
    12: "CALC",
    13: "SCAN",
    14: "LINK",
    15: "SOFT",
    16: "BAD_SUB",
    17: "UDF",
    18: "DISABLE",
    19: "SIMM",
    20: "READ_ACCESS",
    21: "WRITE_ACCESS",
}

SEVERITY_NAMES = {
    NO_ALARM: "NO_ALARM",
    MINOR_ALARM: "MINOR",
    MAJOR_ALARM: "MAJOR",
    INVALID_ALARM: "INVALID",
}

SEVERITY_RANK = {
    "NO_ALARM": 0,
    "MINOR": 1,
    "MAJOR": 2,
    "INVALID": 3,
    "ERROR": 4,
}



def severity_name(severity: Optional[int]) -> str:
    """
    Return EPICS severity name.

    Examples
    --------
    >>> severity_name(3)
    'INVALID'

    >>> severity_name(None)
    'UNKNOWN'
    """
    if severity is None:
        return "UNKNOWN"

    return SEVERITY_NAMES.get(
        severity,
        f"UNKNOWN_SEVERITY({severity})",
    )

def severity_rank(severity: Optional[int]) -> int:
    """
    Convert EPICS severity to sortable rank.

    Examples
    --------
    >>> severity_rank(0)
    0

    >>> severity_rank(2)
    2

    >>> severity_rank(None)
    4
    """
    return SEVERITY_RANK.get(severity_name(severity), SEVERITY_RANK["ERROR"])

def status_name(status: Optional[int]) -> str:
    """
    Return EPICS alarm status name.

    Examples
    --------
    >>> status_name(17)
    'UDF'

    >>> status_name(None)
    'UNKNOWN'
    """
    if status is None:
        return "UNKNOWN"

    return STATUS_NAMES.get( status, f"UNKNOWN_STATUS({status})")

def alarm_info(
    status: Optional[int],
    severity: Optional[int],
) -> str:
    """
    Return formatted EPICS alarm information.
    """
    return (
        f"status={status_name(status)} "
        f"severity={severity_name(severity)}"
    )

def has_alarm( severity: Optional[int]) -> bool:
    """
    True if PV is in alarm state or invalid.
    """
    if severity is None:
        return True

    return severity != NO_ALARM
