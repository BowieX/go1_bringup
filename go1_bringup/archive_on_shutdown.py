#!/usr/bin/env python3
"""Run map archiving after the mapping launch receives a shutdown signal.

ROS 2 launch shutdown is not a good moment to create a brand-new
``ExecuteProcess`` action: depending on timing, the child process can be
cancelled before it has a chance to finish. This small process is started
while the launch is healthy, waits for SIGINT/SIGTERM, then runs archive_map
after FAST-LIO has had time to flush scans.pcd.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time

from go1_bringup.archive_map import main as archive_main


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", default="")
    parser.add_argument("--wait-seconds", type=float, default=30.0)
    args = parser.parse_args()

    stop_event = threading.Event()

    def handle_signal(signum, _frame):
        print(
            f"[archive_on_shutdown] 收到退出信号 {signum}, 准备归档地图...",
            flush=True,
        )
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(
        "[archive_on_shutdown] 已启动, 等待 go1_mapping 退出后归档 scans.pcd.",
        flush=True,
    )
    while not stop_event.is_set():
        time.sleep(0.5)

    argv = ["archive_map", "--wait-seconds", str(args.wait_seconds)]
    if args.label:
        argv.extend(["--label", args.label])

    old_argv = sys.argv
    try:
        sys.argv = argv
        return archive_main()
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
