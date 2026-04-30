#!/usr/bin/env python3
import asyncio
import contextlib
import logging
import signal
import sys
from pathlib import Path

SCRIPT_NAMES = ["bridge.py", "framebuffer-opencv.py"]
RESTART_DELAY_SECONDS = 1.0

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("router")


def get_script_path(script_name: str) -> Path:
    return Path(__file__).resolve().parent / script_name


async def supervise_process(name: str, script_path: Path, stop_event: asyncio.Event) -> None:
    python = sys.executable

    while not stop_event.is_set():
        logger.info("Starting %s: %s", name, script_path)
        process = await asyncio.create_subprocess_exec(
            python,
            str(script_path),
            stdin=asyncio.subprocess.DEVNULL,
            stdout=None,
            stderr=None,
        )

        wait_task = asyncio.create_task(process.wait())
        stop_task = asyncio.create_task(stop_event.wait())

        done, pending = await asyncio.wait(
            {wait_task, stop_task},
            return_when=asyncio.FIRST_COMPLETED,
        )

        if stop_task in done:
            if process.returncode is None:
                logger.info("Stopping %s", name)
                process.terminate()
                try:
                    await asyncio.wait_for(wait_task, timeout=5.0)
                except asyncio.TimeoutError:
                    logger.warning("%s did not exit, killing", name)
                    process.kill()
                    await process.wait()
            break

        # Process exited on its own.
        if wait_task in done:
            return_code = process.returncode
            logger.warning(
                "%s exited with code %s",
                name,
                return_code,
            )

            if stop_event.is_set():
                break

            logger.info("Restarting %s in %.1f seconds", name, RESTART_DELAY_SECONDS)
            await asyncio.sleep(RESTART_DELAY_SECONDS)

        for task in pending:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task


async def shutdown(stop_event: asyncio.Event) -> None:
    logger.info("Shutdown requested")
    stop_event.set()


async def main() -> None:
    stop_event = asyncio.Event()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            # Windows or restricted environment may not support add_signal_handler.
            pass

    tasks = [
        asyncio.create_task(supervise_process(name, get_script_path(name), stop_event))
        for name in SCRIPT_NAMES
    ]

    try:
        await asyncio.gather(*tasks)
    except asyncio.CancelledError:
        await shutdown(stop_event)
    finally:
        if not stop_event.is_set():
            stop_event.set()
        await asyncio.gather(*tasks, return_exceptions=True)
        logger.info("Router stopped")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, exiting")
