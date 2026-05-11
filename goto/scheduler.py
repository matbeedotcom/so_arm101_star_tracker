"""Persistent job queue for time-deferred goto commands.

A scheduled job fires once at its ``at`` timestamp. Jobs are kept in a
single JSON file on disk so they survive restarts — handy on the Pi,
where the tracker may reboot between "I scheduled Polaris for sunset"
and the actual sunset.

The scheduler runs as an asyncio task on the BLE event loop. It picks
the next pending job, sleeps until its fire time (wake-able when a new
job is added), and invokes ``on_fire`` — which the session uses to
submit a goto command via its normal command queue.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from typing import Awaitable, Callable, Optional

log = logging.getLogger("scheduler")


@dataclass
class ScheduledJob:
    id: int
    spec: dict                 # exactly the goto args (target | ra/dec | alt/az)
    at: str                    # ISO 8601 UTC
    state: str = "pending"     # pending | running | done | cancelled | failed
    note: str = ""
    created: str = ""
    fired_at: Optional[str] = None
    error: Optional[str] = None


OnFire = Callable[[ScheduledJob], Awaitable[None] | None]


class Scheduler:
    def __init__(self, store_path: str, on_fire: OnFire):
        self.store_path = store_path
        self.on_fire = on_fire
        self.jobs: list[ScheduledJob] = []
        self._next_id = 1
        self._task: Optional[asyncio.Task] = None
        self._wake: Optional[asyncio.Event] = None
        self._listeners: list[Callable[[], None]] = []
        self._load()

    # ── lifecycle ──

    async def start(self) -> None:
        if self._task is not None:
            return
        self._wake = asyncio.Event()
        self._task = asyncio.create_task(self._run(), name="scheduler")
        log.info("scheduler started (%d jobs queued)",
                 sum(1 for j in self.jobs if j.state == "pending"))

    async def stop(self) -> None:
        if self._task is None:
            return
        self._task.cancel()
        try:
            await self._task
        except (asyncio.CancelledError, Exception):
            pass
        self._task = None

    # ── public API ──

    def on_change(self, cb: Callable[[], None]) -> Callable[[], None]:
        self._listeners.append(cb)
        return lambda: self._listeners.remove(cb)

    def add(self, spec: dict, at: str, note: str = "") -> ScheduledJob:
        # Normalise ``at`` to UTC ISO with explicit suffix so the file
        # is unambiguous when read back from a different process.
        at_norm = _normalise_iso(at)
        job = ScheduledJob(
            id=self._next_id, spec=dict(spec), at=at_norm, note=note,
            created=_now_iso(),
        )
        self._next_id += 1
        self.jobs.append(job)
        self._save()
        if self._wake is not None:
            self._wake.set()
        self._fire_changed()
        return job

    def cancel(self, job_id: int) -> bool:
        for j in self.jobs:
            if j.id == job_id and j.state == "pending":
                j.state = "cancelled"
                self._save()
                if self._wake is not None:
                    self._wake.set()
                self._fire_changed()
                return True
        return False

    def list_jobs(self, include_done: bool = False) -> list[dict]:
        out = []
        for j in self.jobs:
            if not include_done and j.state in ("done", "cancelled", "failed"):
                continue
            out.append(asdict(j))
        out.sort(key=lambda d: d["at"])
        return out

    # ── internals ──

    async def _run(self) -> None:
        while True:
            assert self._wake is not None
            self._wake.clear()
            pending = [j for j in self.jobs if j.state == "pending"]
            if not pending:
                await self._wake.wait()
                continue

            pending.sort(key=lambda j: j.at)
            next_job = pending[0]
            try:
                fire_time = _parse_iso(next_job.at)
            except Exception as e:
                log.warning("bad job %d at=%r: %s", next_job.id, next_job.at, e)
                next_job.state = "failed"
                next_job.error = f"bad timestamp: {e}"
                self._save(); self._fire_changed()
                continue

            delay = (fire_time - datetime.now(timezone.utc)).total_seconds()
            if delay > 0:
                # Cap wait so we re-check periodically (clock jumps, etc.).
                try:
                    await asyncio.wait_for(self._wake.wait(), timeout=min(delay, 3600))
                except asyncio.TimeoutError:
                    pass
                continue

            # Fire — and refuse to fire something more than 5 minutes overdue,
            # which usually means the scheduler was offline for a while.
            if delay < -300:
                next_job.state = "failed"
                next_job.error = f"missed by {-delay:.0f}s"
                self._save(); self._fire_changed()
                log.warning("job %d missed fire time by %.0fs", next_job.id, -delay)
                continue

            next_job.state = "running"
            next_job.fired_at = _now_iso()
            self._save(); self._fire_changed()
            try:
                r = self.on_fire(next_job)
                if asyncio.iscoroutine(r):
                    await r
                next_job.state = "done"
            except Exception as e:
                next_job.state = "failed"
                next_job.error = str(e)
                log.exception("job %d fire failed", next_job.id)
            self._save(); self._fire_changed()

    def _fire_changed(self) -> None:
        for cb in list(self._listeners):
            try: cb()
            except Exception: pass

    # ── persistence ──

    def _load(self) -> None:
        if not os.path.exists(self.store_path):
            return
        try:
            with open(self.store_path) as f:
                data = json.load(f)
        except Exception as e:
            log.warning("schedule file unreadable: %s — starting fresh", e)
            return
        for raw in data.get("jobs", []):
            try:
                self.jobs.append(ScheduledJob(**raw))
            except Exception as e:
                log.warning("skipping corrupt job %r: %s", raw, e)
        if self.jobs:
            self._next_id = max(j.id for j in self.jobs) + 1

    def _save(self) -> None:
        try:
            os.makedirs(os.path.dirname(self.store_path) or ".", exist_ok=True)
            tmp = self.store_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"jobs": [asdict(j) for j in self.jobs]}, f, indent=2)
            os.replace(tmp, self.store_path)
        except Exception as e:
            log.warning("save failed: %s", e)


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _normalise_iso(s: str) -> str:
    # Accept "...Z" or "+00:00" or naive (treat as UTC). Always emit
    # "...Z" form.
    parsed = _parse_iso(s)
    return parsed.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _parse_iso(s: str) -> datetime:
    s2 = s.strip()
    if s2.endswith("Z"):
        s2 = s2[:-1] + "+00:00"
    dt = datetime.fromisoformat(s2)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt
