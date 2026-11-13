"""Shared bounded scope cleanup for synchronous and asynchronous facades."""
import asyncio
import time

from .errors import CleanupFailed


async def exit_scope(close, exc_info, timeout=60.0):
    """Retain body and cleanup errors; shield one finite cleanup attempt."""
    body_error = exc_info[1] if len(exc_info) > 1 else None
    cleanup = asyncio.create_task(close())
    deadline = time.monotonic()+timeout
    interrupted = False
    try:
        while True:
            try:
                await asyncio.wait_for(asyncio.shield(cleanup), max(0.,deadline-time.monotonic()))
                break
            except asyncio.CancelledError:
                if cleanup.cancelled():
                    raise CleanupFailed("cleanup itself was cancelled; state is unverified")
                interrupted = True
        if interrupted:
            raise asyncio.CancelledError()
    except asyncio.TimeoutError as exc:
        cleanup.cancel()
        failure = CleanupFailed("cleanup deadline expired; physical state is unverified",
                                details={"timeout": timeout, "physical_stop": "unverified"})
        if body_error is None:
            raise failure from exc
        body_error.add_note(str(failure))
        body_error.cleanup_error = failure
    except Exception as failure:
        if body_error is None:
            raise
        body_error.add_note(f"Cleanup also failed: {failure}")
        body_error.cleanup_error = failure
