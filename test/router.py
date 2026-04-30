#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


CHECK_AUX_FILE = "controller.py"
FRAMEBUFFER_FILE = "framebuffer.py"

STOP_TIMEOUT_SEC = 2.0

running = True


def on_signal(signum, frame):
    global running
    running = False


def parse_cores(value: str) -> set[int]:
    return {int(core) for core in value.split(",") if core.strip()}


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


def run(check_cores: set[int], framebuffer_cores: set[int]) -> None:
    validate_file(CHECK_AUX_FILE)
    validate_file(FRAMEBUFFER_FILE)

    validate_cores(check_cores)
    validate_cores(framebuffer_cores)

    check_aux = start_process(CHECK_AUX_FILE, check_cores)
    framebuffer = start_process(FRAMEBUFFER_FILE, framebuffer_cores)

    try:
        while running:
            time.sleep(1.0)
    finally:
        stop_process(framebuffer)
        stop_process(check_aux)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-cores", default="3")
    parser.add_argument("--framebuffer-cores", default="1,2")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    run(
        parse_cores(args.check_cores),
        parse_cores(args.framebuffer_cores),
    )


if __name__ == "__main__":
    main()