#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


CHECK_AUX_FILE = "controller.py"
CALCULATE_CORRECTION_FILE = "calculate_correction.py"
FRAMEBUFFER_FILE = "framebuffer.py"

CHECK_CORES = {3}
FRAMEBUFFER_CORES = {1, 2}
CORRECTION_CORES = {0}

STOP_TIMEOUT_SEC = 2.0

running = True


def on_signal(signum, frame):
    global running
    running = False


def validate_file(path: str) -> None:
    if not Path(path).is_file():
        raise FileNotFoundError(path)


def validate_cores(cores: set[int]) -> None:
    cpu_count = os.cpu_count() or 1

    for core in cores:
        if core < 0 or core >= cpu_count:
            raise ValueError(f"invalid CPU core: {core}")


def setup_process(cores: set[int]) -> None:
    os.setsid()
    os.sched_setaffinity(0, cores)


def start_process(script: str, cores: set[int]) -> subprocess.Popen:
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    proc = subprocess.Popen(
        [sys.executable, "-u", script],
        stdout=None,
        stderr=None,
        env=env,
        preexec_fn=lambda: setup_process(cores),
    )

    print(f"{script}: pid={proc.pid}, cpus={sorted(cores)}", flush=True)
    return proc


def stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return

    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=STOP_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def run() -> None:
    validate_file(CHECK_AUX_FILE)
    validate_file(CALCULATE_CORRECTION_FILE)
    validate_file(FRAMEBUFFER_FILE)

    validate_cores(CHECK_CORES)
    validate_cores(FRAMEBUFFER_CORES)
    validate_cores(CORRECTION_CORES)

    check_aux = start_process(CHECK_AUX_FILE, CHECK_CORES)
    correction = start_process(CALCULATE_CORRECTION_FILE, CORRECTION_CORES)
    framebuffer = start_process(FRAMEBUFFER_FILE, FRAMEBUFFER_CORES)

    try:
        while running:
            time.sleep(1.0)
    finally:
        stop_process(framebuffer)
        stop_process(correction)
        stop_process(check_aux)


def main() -> None:
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    run()


if __name__ == "__main__":
    main()