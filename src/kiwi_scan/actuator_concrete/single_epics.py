# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from kiwi_scan.actuator.single import (
    AbstractActuator,
    MonitorCallback,
    _close_created_pvs,
)
from kiwi_scan.datamodels import ActuatorConfig
from kiwi_scan.epics_wrapper import EpicsPV, PutCompletion

logger = logging.getLogger(__name__)

class EpicsActuator(AbstractActuator):
    """
    An EPICS-based actuator implementation using the EpicsPV wrapper.
    """

    def __init__(self, config: ActuatorConfig):
        super().__init__(config)

        # Latest put request, shared with status checks and stop().
        self._pending_put: Optional[PutCompletion] = None
        # PVs owned by this actuator; close() releases them in reverse order
        created_pvs: List[EpicsPV] = []
        self._owned_pvs = created_pvs

        def create_pv(pvname: str, **kwargs):
            pv = EpicsPV(pvname, **kwargs)
            created_pvs.append(pv)
            return pv

        try:
            self.pv = create_pv(config.pv, queueing_delay=self.q_delay) if config.pv else None
            self.ca_timeout = config.ca_timeout if config.ca_timeout else 1.0
            self.auto_monitor = config.auto_monitor 
            logger.debug(f"Actuator CA settings:  ca_timeout: {self.ca_timeout}, auto_monitor: {self.auto_monitor}")
            self.rel_pv = create_pv(config.rel_pv, queueing_delay=self.q_delay) if config.rel_pv else None
            self.rb_pv = create_pv(config.rb_pv, auto_monitor=self.auto_monitor, queueing_delay=0.0) if config.rb_pv else None
            self.cmd_pv = create_pv(config.cmd_pv, queueing_delay=self.q_delay) if config.cmd_pv else None
            self.cmdvel_pv = create_pv(config.cmdvel_pv, queueing_delay=self.q_delay) if config.cmdvel_pv else None
            self.start_pv = create_pv(config.start_pv, queueing_delay=self.q_delay) if config.start_pv else None
            self.stop_pv = create_pv(config.stop_pv, queueing_delay=self.q_delay) if config.stop_pv else None
            self.status_pv = create_pv(config.status_pv, auto_monitor=self.auto_monitor, queueing_delay=0.0) if config.status_pv else None
            self.velocity_pv = create_pv(config.velocity_pv, queueing_delay=self.q_delay) if config.velocity_pv else None
            self.get_velocity_pv = create_pv(config.get_velocity_pv) if config.get_velocity_pv else None
            # pvname -> list of callback indices (pyepics returns an int id per add_callback)
            self._epics_cb_indices: Dict[str, List[int]] = {}

            self.jog_velocity_pv = None
            self.jog_command_pv = None
            jog_cfg = self.config.jog
            if jog_cfg:
                self.jog_velocity_pv = create_pv(jog_cfg.velocity_pv, queueing_delay=self.q_delay) if jog_cfg.velocity_pv else None
                self.jog_command_pv = create_pv(jog_cfg.command_pv, queueing_delay=self.q_delay) if jog_cfg.command_pv else None

            # Config parameters
            self.in_pos_band = config.in_position_band
            self.dwell_time = config.dwell_time
            self.ready_value = config.ready_value
            self.ready_bitmask = config.ready_bitmask
            self.q_delay = config.queueing_delay
            self.startup_timeout = config.startup_timeout
            self.backlash = config.backlash
            self.velocity = config.velocity
            self.start_command = config.start_command
            self.put_timeout = config.put_timeout
            if self.put_timeout > 0:
                logger.debug("[%s] main-PV put completion enabled (timeout %.3fs)", self.pvname, self.put_timeout)

            # Validate PV connections
            self._check_pvs()
        except BaseException:
            _close_created_pvs(created_pvs)
            raise

    def close(self) -> None:
        """Remove all monitors and close every PV owned by the actuator. Idempotent."""
        pending = self._pending_put
        if pending is not None:
            pending.cancel()
        self.clear_monitors()
        owned, self._owned_pvs = self._owned_pvs, []
        _close_created_pvs(owned)

    def set_queueing_delay(self, delay: float) -> None:
        """Update actuator pacing and every owned command-PV put delay."""
        super().set_queueing_delay(delay)
        for pv in (
            self.pv, self.rel_pv, self.cmd_pv, self.cmdvel_pv,
            self.start_pv, self.stop_pv, self.velocity_pv,
            self.jog_velocity_pv, self.jog_command_pv,
        ):
            if pv is not None:
                pv.queueing_delay = self.q_delay

    # --------------------- monitor backend ---------------------------------
    def supports_monitors(self) -> bool:
        return True

    def add_monitor(
        self,
        pvname: str,
        user_callback: Optional[MonitorCallback] = None,
        **kwargs: Any,
    ) -> Any:
        """
        Subscribe to EPICS PV updates (CA monitor) and route them to _dispatch_pv_update()
        so scan/subscription callbacks receive PvEvent objects.
        """
        with self._monitor_lock:
            # Create the EPICS monitor PV only once
            if pvname not in self._monitors:
                pvname_outer = pvname

                # Create a dedicated PV for monitoring with auto_monitor enabled
                if hasattr(EpicsPV, "create_monitor"):
                    mon = EpicsPV.create_monitor(
                        pvname,
                        timeout=float(kwargs.get("timeout", 1.0)),
                        queueing_delay=float(kwargs.get("queueing_delay", self.q_delay)),
                        auto_monitor=True)
                else:
                    # fallback for tests / pyepics compatible API
                    mon = EpicsPV(
                        pvname,
                        timeout=float(kwargs.get("timeout", 1.0)),
                        queueing_delay=float(kwargs.get("queueing_delay", self.q_delay)),
                        auto_monitor=True)

                def _on_ca_event(pvname=None, value=None, **kw):
                    name = pvname if pvname is not None else pvname_outer

                    # Keep any provided kw fields; also allow callers to tag source
                    if "source" not in kw:
                        kw["source"] = "epics_monitor"

                    # IMPORTANT: route through actuator dispatcher (creates PvEvent and fans out)
                    self._dispatch_pv_update(name, value, **kw)

                # EpicsPV.add_callback returns callback index from pyepics
                logger.info(f"ACT->add_monitor: Add callback to PV {pvname}")
                cb_idx = mon.add_callback(_on_ca_event, run_now=False, with_ctrlvars=False)
                self._monitors[pvname] = mon
                self._epics_cb_indices[pvname] = [cb_idx]

            # Ensure callbacks list exists even if caller only wants to "listen"
            self._monitor_callbacks.setdefault(pvname, [])

        # Optional user callback gets PvEvent
        if user_callback is not None:
            self.on_pv_event(pvname, user_callback)

        return self._monitors.get(pvname)

    def remove_monitor(self, pvname: str) -> None:
        """ Remove CA callbacks for the monitored PV and clear bookkeeping. """
        with self._monitor_lock:
            mon = self._monitors.get(pvname)
            cb_indices = list(self._epics_cb_indices.get(pvname, []))

        if mon is not None:
            for idx in cb_indices:
                try:
                    mon.remove_callback(idx)
                except Exception:
                    logger.exception("[EPICS] remove_callback failed for %s idx=%r", pvname, idx)
            try:
                mon.close()
            except Exception:
                logger.debug("[EPICS] close failed for %s", pvname, exc_info=True)

        with self._monitor_lock:
            self._epics_cb_indices.pop(pvname, None)

        super().remove_monitor(pvname)

    def clear_monitors(self) -> None:
        """ Remove all EPICS monitors for this actuator. """
        with self._monitor_lock:
            pvs = list(self._monitors.keys())
        for pv in pvs:
            self.remove_monitor(pv)

    def _dispatch_pv_update(self, pvname: str, value: Any, **kw: Any):
        """ Only record/callback for PVs that are still monitored/listened to. """
        with self._monitor_lock:
            has_monitor = pvname in self._monitors
            has_listeners = bool(self._monitor_callbacks.get(pvname))
        if not (has_monitor or has_listeners):
            return None
        return super()._dispatch_pv_update(pvname, value, **kw)

    @property
    def pvname(self) -> str:
        if self.pv:
            return self.pv.pvname
        return "<setter PV not configured>"

    @property
    def rbv(self) -> Optional[Any]:
        """ Shortcut property to get the readback value.  """
        
        if self.rb_pv:
            val = self.rb_pv.get(use_monitor=True)
            if val is None:
                val = self.rb_pv.get(timeout=self.ca_timeout) # fallback poll once
            return val
        return None

    @rbv.setter
    def rbv(self, value: Any) -> None:
        """ Shortcut property to set the readback value (for testing or simulation).  """
        if self.rb_pv:
            self.rb_pv.put(value)
        else:
            raise AttributeError("Read-back PV not configured; cannot set rbv")

    @property
    def cmdv(self) -> Optional[Any]:
        """ Shortcut property to get the commanded position value.  """
        if self.cmd_pv:
            val = self.cmd_pv.get(use_monitor=True) 
            if val is None: 
                val = self.cmd_pv.get(timeout=self.ca_timeout) # fallback once
            return val
        return None

    @cmdv.setter
    def cmdv(self, value: Any) -> None:
        """ Shortcut property to set the commanded position value (for testing or simulation).  """
        if self.cmd_pv:
            self.cmd_pv.put(value)
        else:
            raise AttributeError("Command PV not configured; cannot set cmdv")
    
    @property
    def cmdvelv(self) -> Optional[Any]:
        """ Shortcut property to get the commanded position value.  """
        if self.cmdvel_pv:
            val = self.cmdvel_pv.get(use_monitor=True) 
            if val is None: 
                val = self.cmdvel_pv.get(timeout=self.ca_timeout) # fallback once
            return val
        return None

    @cmdvelv.setter
    def cmdvelv(self, value: Any) -> None:
        """
        Shortcut property to set the commanded position value (for testing or simulation).
        """
        if self.cmdvel_pv:
            self.cmdvel_pv.put(value)
        else:
            raise AttributeError("Command PV not configured; cannot set cmdv")
    
    def _check_pvs(self) -> None:
        """Ensure all required PVs are connected and writable."""
        if self.pv:
            self.pv.check_pv()
        else:
            logger.warning("Creating EPICS actuator without setter PV")
        if self.start_pv:
            self.start_pv.check_pv()
        if self.velocity_pv:
            self.velocity_pv.check_pv()
        if self.stop_pv:
            self.stop_pv.check_pv()

    def _wait_for_condition(
        self,
        condition: Callable[[], bool],
        timeout: Optional[float] = None,
        interval: float = 0.1,
        msg: str = "Timeout waiting for condition",
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        """
        Wait until `condition()` returns True.
        Individual failed condition function reads are handled by the condition function (and return False). 
        This method keeps polling until the configured timeout expires.

        Returns: 
            True if the condition was observed. 
            False on timeout.
        """
        start = time.time()
        while not condition():
            if stop_event is not None and stop_event.is_set():
                logger.info("Stop requested while waiting for actuator condition")
                return False

            if timeout is not None and (time.time() - start) > timeout:
                logger.warning(msg)
                return False

            if stop_event is not None:
                if stop_event.wait(interval):
                    logger.info("Stop requested while waiting for actuator condition")
                    return False
            else:
                time.sleep(interval)

        return True
    
    def start_actuator(self) -> None:
        if self.start_pv:
            success = self.start_pv.put(self.start_command)
            if not success:
                logger.error(f"Failed to start actuator via {self.start_pv.pvname}")

    def set_velocity(self, velocity: float) -> None:
        self.velocity = velocity
        if self.velocity_pv:
            success = self.velocity_pv.put(velocity)
            if not success:
                logger.error(f"Failed to set velocity via {self.velocity_pv.pvname}")
        logger.info(f"Velocity set to {self.velocity}")

    def get_velocity(self) -> Optional[float]:
        if self.get_velocity_pv:
            val = self.get_velocity_pv.get(use_monitor=True) 
            if val is None: 
                val = self.get_velocity_pv.get(timeout=self.ca_timeout) # fallback once
            return val
        return None

    def _issue_move(self, position: float) -> None:
        """
        Write the main setpoint. This method never waits for motion.

        With ``put_timeout > 0`` the write uses a CA put callback, for records with asynchronous processing. 
        The request is kept in ``self._pending_put``: while it is pending, 
        ``is_ready()`` is False and ``is_moving()`` is True, and ``wait_until_done()`` waits for it.
        A configured ``start_pv`` is written after submitting the setpoint and before any completion wait. 
        """
        if not self.pv:
            logger.error("Cannot issue move to %r: setter PV is not configured", position)
            return

        logger.info("[%s] move to %f", self.pvname, position)
        if self.put_timeout > 0:
            previous = self._pending_put
            if previous is not None and previous.pending:
                logger.debug("[%s] new move while previous put is still processing", self.pvname)
            completion = self.pv.put_with_completion(position, timeout=self.put_timeout)
            self._pending_put = completion
            success = not completion.failed
        else:
            success = self.pv.put(position)
        if not success:
            logger.error("Failed to write position to %s", self.pvname)
        self.start_actuator()
        time.sleep(self.q_delay)

    def move(self, position: float) -> None:
        self._issue_move(position)

    def _issue_rel_move(self, delta: float) -> None:
        """Issue a relative move.

        If config.rel_pv exists and is connected, write delta to it.
        Otherwise compute an absolute target from rbv and use the normal move PV.
        """
        if self.rel_pv is not None:
            logger.info(f"[{self.pvname}] rel-move by {delta}")
            # rel_pv writes are plain puts
            self._pending_put = None
            success = self.rel_pv.put(delta)
            if not success:
                logger.error("Failed to write relative move %r to %s", delta, self.rel_pv.pvname)
            self.start_actuator()
            time.sleep(self.q_delay)
            return

        # Fallback: compute absolute target from readback
        cur = self.rbv
        if cur is None:
            logger.error( f"Relative move requested but no rel_pv configured/available and rbv is None for actuator '{self.pvname}'.")
            return 
        try:
            target = float(cur) + float(delta)
        except (TypeError, ValueError) as exc:
            logger.error("Failed to compute absolute target from rbv=%r and delta=%r: %s", cur, delta, exc)
            return        
        logger.info(f"[{self.pvname}] rel-move fallback: rbv={cur} delta={delta} -> target={target}")
        self._issue_move(target)

    def run_move(
        self,
        position: float,
        sync: bool = True,
        wait_startup: bool = False,
    ) -> None:
        self.move(position)
        if sync:
            self.wait_until_done(position)
        elif wait_startup:
            self.wait_for_startup()

    def run_rel_move(
        self,
        delta: float,
        sync: bool = True,
        wait_startup: bool = False,
    ) -> None:
        """Relative move; if we fall back to absolute moves, we wait on the computed target."""
        if self.rel_pv is not None:
            cur = self.rbv
            self.rel_move(delta)
            if sync:
                try:
                    if cur is not None and self.in_pos_band >= 0:
                        self.wait_until_done(float(cur) + float(delta))
                    else:
                        self.wait_for_startup_and_done()
                        self.dwell()
                except Exception:  # noqa: BLE001   - communication may fail temporarily
                    self.wait_for_startup_and_done()
                    self.dwell()
            elif wait_startup:
                self.wait_for_startup()
            return

        cur = self.rbv
        if cur is None:
            logger.error( f"Relative move requested but no rel_pv configured/available and rbv is None for actuator '{self.pvname}'.")
            return
        target = float(cur) + float(delta)
        self._issue_move(target)
        if sync:
            self.wait_until_done(target)
        elif wait_startup:
            self.wait_for_startup()

    def rel_move(self, delta: float) -> None:
        self._issue_rel_move(delta)

    def jog(self, velocity: float, sync: bool = True) -> bool:
        """Write jog settings, returning False if an output write fails."""
        logger.debug("Jog with velocity %s", velocity)
        jog_cfg = self.config.jog
        if not jog_cfg:
            raise ValueError("Jog feature is not configured for this actuator")
        
        # 1) If a velocity PV is provided, write the desired velocity
        if self.jog_velocity_pv is not None:
            if jog_cfg.abs_velocity:
                cmd_velocity = abs(velocity)
            else:
                cmd_velocity = velocity
            success = self.jog_velocity_pv.put(cmd_velocity)
            if not success:
                logger.error("Failed to set velocity via %s", self.jog_velocity_pv.pvname)
                return False
        
        # 2) If a command PV is provided, determine the command value
        if self.jog_command_pv is not None:
            if jog_cfg.command_pos is not None and jog_cfg.command_neg is not None:
                cmd = jog_cfg.command_pos if velocity >= 0 else jog_cfg.command_neg
            else:
                # configured command value: zero
                cmd = jog_cfg.command_pos if jog_cfg.command_pos is not None else 1.0
            success = self.jog_command_pv.put(cmd)
            if not success:
                logger.error("Failed to set %s", self.jog_command_pv.pvname)
                return False
        if sync:
            self.wait_for_startup_and_done()
        return True
    
    def _read_status_value(self) -> Optional[Any]:
        """Read the status PV using monitor cache with one CA fallback poll."""
        if not self.status_pv:
            return None

        val = self.status_pv.get(use_monitor=True)
        if val is None:
            val = self.status_pv.get(timeout=self.ca_timeout)
        return val

    def _status_value_is_ready(self, val: Any) -> bool:
        """Decode a concrete status-PV value into ready/not-ready."""
        mask = getattr(self, "ready_bitmask", 0)
        logger.debug("is_ready(): mask=%r val=%r", mask, val)

        if mask:
            try:
                status = int(val)
                mask = int(mask)
                # ready_value may be int or a string such as "0x0B22".
                ready_val = (
                    int(self.ready_value, 0)
                    if isinstance(self.ready_value, str)
                    else int(self.ready_value)
                )
                logger.debug(
                    "is_ready(): status=%r mask=%r ready_val=%r",
                    status,
                    mask,
                    ready_val,
                )
                return (status & mask) == ready_val
            except (TypeError, ValueError):
                logger.debug("is_ready(): failed bitmask evaluation", exc_info=True)
                # Fall back to simple comparison below.

        try:
            return float(val) == float(self.ready_value)
        except (TypeError, ValueError):
            return str(val).strip() == str(self.ready_value).strip()

    def _ready_state(self) -> Optional[bool]:
        """
        Return the actuator state..

        Returns:
            True:  status was read successfully and decodes as ready.
            False: status was read successfully and decodes as not ready.
            None:  status is unknown, for example during a short CA disconnect.

        A pending main-PV put completion counts as not ready (busy).
        """
        pending = self._pending_put
        if pending is not None and pending.pending:
            return False

        if not self.status_pv:
            return True

        val = self._read_status_value()
        if val is None:
            logger.debug(f"[{self.pvname}] status state unknown")
            return None

        return self._status_value_is_ready(val)

    def is_ready(self) -> bool:
        """Return True only when the status PV confirms the ready state."""
        return self._ready_state() is True

    def is_moving(self) -> bool:
        """
        Return True only when the status PV confirms a not-ready state.

        Unknown status is not treated as moving.  This prevents a
        short CA disconnect from satisfying the startup wait by accident.
        """
        return self._ready_state() is False

    def in_position_check(
        self,
        target: float,
        timeout: float = 0,
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        if self.in_pos_band < 0 or not self.rb_pv:
            return True

        start = time.time()

        while True:
            if stop_event is not None and stop_event.is_set():
                logger.info(f"[{self.pvname}] in-position check aborted by stop_event")
                return False

            current = self.rb_pv.get(timeout=self.ca_timeout)
            if current is None:
                logger.warning("Readback PV returned None")
                return True

            if abs(current - target) <= self.in_pos_band:
                return True

            if timeout and (time.time() - start) >= timeout:
                return False

            if stop_event is not None:
                if stop_event.wait(0.1):
                    logger.info(f"[{self.pvname}] in-position check aborted by stop_event")
                    return False
            else:
                time.sleep(0.1)

    def is_in_position(self, target, in_position_band):
        rb_pv = self.rb_pv
        if rb_pv is None:
            return True
        current = rb_pv.get(timeout=self.ca_timeout)
        if current is None:
            return True
        return abs(current - target) <= in_position_band
    
    def dwell(self) -> None:
        time.sleep(self.dwell_time)

    def wait_for_startup(
        self,
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        """
        Wait until actuator motion has started.
        Returns: 
            True if startup was observed or no status PV configured.
            False on timeout or when stop_event is set. 
        """
        if self._stop_requested(stop_event):
            logger.info(f"[{self.pvname}] startup wait aborted before start")
            return False

        if not self.status_pv:
            logger.debug(f"[{self.pvname}] no status PV configured; cannot observe startup")
            return True

        logger.debug(f"[{self.pvname}] waiting for move to start")
        started = self._wait_for_condition(self.is_moving, self.startup_timeout,
            msg=(f"[{self.pvname}] move start was not observed within {self.startup_timeout}s"), stop_event=stop_event)

        if self._stop_requested(stop_event):
            logger.info(f"[{self.pvname}] startup wait aborted")
            return False

        if not started:
            logger.warning( f"[{self.pvname}] move start was not observed")

        return bool(started)

    def wait_for_startup_and_done(self, stop_event: Optional[threading.Event] = None) -> None:
        self.wait_for_startup(stop_event=stop_event)
        if stop_event is not None and stop_event.is_set():
            return

        logger.debug(f"[{self.pvname}] waiting for ready state")
        # TODO: Add configurable timeout
        self._wait_for_condition(self.is_ready, stop_event=stop_event)
    
    def _stop_requested(self, stop_event: Optional[threading.Event]) -> bool:
        return stop_event is not None and stop_event.is_set()


    def _dwell_interruptible(
        self,
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        """Return False if dwell was interrupted by stop_event."""
        if self.dwell_time <= 0:
            return True

        logger.debug(f"[{self.pvname}] dwell for {self.dwell_time}s")

        if stop_event is not None:
            return not stop_event.wait(self.dwell_time)

        self.dwell()
        return True
    
    def wait_until_done(self, position: float, stop_event: Optional[threading.Event] = None) -> None:
        has_status = bool(self.status_pv)
        has_band = self.in_pos_band >= 0
        t0 = time.time()

        try:
            if self._stop_requested(stop_event):
                logger.info(f"[{self.pvname}] wait aborted before start")
                return

            # A completed put with completion means motion has already started and ended.
            put_completed = False
            completion = self._pending_put
            if completion is not None:
                put_completed = completion.wait(stop_event=stop_event)
                if self._stop_requested(stop_event) or completion.cancelled:
                    logger.info("[%s] wait aborted during put completion", self.pvname)
                    return

            if has_status and put_completed:
                logger.debug("[%s] put completed; skipping startup wait", self.pvname)
                self._wait_for_condition(self.is_ready, stop_event=stop_event)
                if self._stop_requested(stop_event):
                    logger.info(f"[{self.pvname}] wait aborted after status wait")
                    return
            elif has_status:
                self.wait_for_startup_and_done(stop_event=stop_event)
                if self._stop_requested(stop_event):
                    logger.info(f"[{self.pvname}] wait aborted after status wait")
                    return

            if has_band:
                logger.debug(f"[{self.pvname}] waiting in-band")
                if not self.in_position_check(position, stop_event=stop_event):
                    if self._stop_requested(stop_event):
                        logger.info(f"[{self.pvname}] in-band wait aborted")
                        return
                    logger.warning(f"{self.pvname} never reached in-band position")

                if self._stop_requested(stop_event):
                    logger.info(f"[{self.pvname}] wait aborted after in-band check")
                    return

            if has_status or has_band or self.dwell_time > 0:
                if not self._dwell_interruptible(stop_event):
                    logger.info(f"[{self.pvname}] dwell interrupted")
                    return
            else:
                logger.info(f"[{self.pvname}] no wait conditions")

        finally:
            elapsed = time.time() - t0
            logger.info(f"[{self.pvname}] done in {elapsed:.3f}s")
            self._last_move_time = elapsed


    def stop(self) -> None:
        # Release waiters and clear the busy state first; 
        # stop() may be called from another thread (e.g. Ctrl-C handling).
        pending = self._pending_put
        if pending is not None:
            pending.cancel()
        if self.stop_pv:
            success = self.stop_pv.put(self.config.stop_command)
            if not success:
                logger.error(f"Failed to stop actuator via {self.stop_pv.pvname}")
        else:
            logger.debug("Stop PV not defined, no action taken")
