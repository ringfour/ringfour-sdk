"""The sync API's event loop host.

One background thread runs an asyncio loop owning the connection; every
blocking method in the sync API submits a coroutine here and waits.
Timeouts are enforced inside the aio layer (which raises ringfour's
TimeoutError), so ``run`` itself waits indefinitely.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any, Coroutine


class Runner:
    """Owns one event loop thread. Create, run coroutines, stop."""

    def __init__(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, name="ringfour-io", daemon=True
        )
        self._thread.start()

    def run(self, coro: Coroutine) -> Any:
        """Run a coroutine on the loop and block for its result."""
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result()

    def stop(self) -> None:
        """Stop the loop and join the thread. Idempotent."""
        if self._loop.is_closed():
            return
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)
        if not self._thread.is_alive():
            self._loop.close()
