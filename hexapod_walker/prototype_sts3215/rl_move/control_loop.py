"""Absolute-deadline control loop scheduler."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class TickTiming:
    actual_dt: float = 0.0
    work_s: float = 0.0
    overrun: bool = False
    tick_index: int = 0


@dataclass
class LoopStats:
    ticks: int = 0
    overruns: int = 0
    sum_dt: float = 0.0
    max_dt: float = 0.0
    max_work: float = 0.0
    last: TickTiming = field(default_factory=TickTiming)


class ControlLoop:
    """Call ``work()`` every ``1/hz`` seconds against absolute deadlines."""

    def __init__(self, hz: float = 50.0):
        self.period = 1.0 / float(hz)
        self.stats = LoopStats()
        self._t_next: float | None = None
        self._t_prev: float | None = None
        self._running = False

    def reset(self) -> None:
        self.stats = LoopStats()
        self._t_next = None
        self._t_prev = None

    def run(self, work: Callable[[TickTiming], None], *,
            duration_s: float | None = None,
            max_ticks: int | None = None,
            should_stop: Callable[[], bool] | None = None) -> LoopStats:
        self._running = True
        t0 = time.monotonic()
        self._t_next = t0
        self._t_prev = None
        n = 0
        while self._running:
            if duration_s is not None and (time.monotonic() - t0) >= duration_s:
                break
            if max_ticks is not None and n >= max_ticks:
                break
            if should_stop is not None and should_stop():
                break

            self._t_next += self.period
            t_start = time.monotonic()
            actual_dt = 0.0 if self._t_prev is None else (t_start - self._t_prev)
            self._t_prev = t_start

            tick = TickTiming(actual_dt=actual_dt, tick_index=n)
            work(tick)
            work_s = time.monotonic() - t_start
            tick.work_s = work_s

            remaining = self._t_next - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
                tick.overrun = False
            else:
                tick.overrun = True
                self.stats.overruns += 1
                # Resync deadline so one slow tick does not cascade forever.
                self._t_next = time.monotonic()

            self.stats.ticks += 1
            self.stats.sum_dt += actual_dt
            self.stats.max_dt = max(self.stats.max_dt, actual_dt)
            self.stats.max_work = max(self.stats.max_work, work_s)
            self.stats.last = tick
            n += 1

        self._running = False
        return self.stats

    def stop(self) -> None:
        self._running = False
