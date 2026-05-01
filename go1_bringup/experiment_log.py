#!/usr/bin/env python3
"""Create a reproducible experiment session folder.

This utility does not start hardware. It creates a fixed directory layout,
metadata file, geometry template, and command checklist so field data is not
mixed across repeated runs.
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import subprocess
from datetime import datetime
from pathlib import Path


KINDS = (
    "slam_degradation",
    "slam_normal",
    "mapping",
    "nav_baseline",
    "nav_improved",
    "custom",
)


def safe_name(text: str) -> str:
    text = text.strip()
    if not text:
        return ""
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text)
    return text.strip("_")


def run_short(cmd: list[str]) -> str:
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT).strip()
    except Exception as exc:  # pragma: no cover - diagnostic best effort
        return f"unavailable: {exc}"


def write_metadata(path: Path, args: argparse.Namespace, session_name: str) -> None:
    lines = [
        f"session: {session_name}",
        f"kind: {args.kind}",
        f"created_at: {datetime.now().isoformat(timespec='seconds')}",
        f"operator: {args.operator}",
        f"location: {args.location}",
        f"route: {args.route}",
        f"notes: {args.notes}",
        f"host: {platform.node()}",
        f"platform: {platform.platform()}",
        f"ros_distro: {os.environ.get('ROS_DISTRO', '')}",
        f"git_go1_bringup: {run_short(['git', '-C', 'go1_bringup', 'rev-parse', '--short', 'HEAD'])}",
        f"git_fast_lio: {run_short(['git', '-C', 'FAST_LIO', 'rev-parse', '--short', 'HEAD'])}",
        "",
        "field_checklist:",
        "  hardware_topics_ok: false",
        "  tf_ok: false",
        "  timestamp_ok: false",
        "  fast_lio_odometry_ok: false",
        "  bag_recorded: false",
        "  replay_baseline_done: false",
        "  replay_improved_done: false",
        "  replay_always_done: false",
        "  evaluation_done: false",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_geometry_template(path: Path) -> None:
    path.write_text(
        "# label,measured_distance_m,start_x,start_y,end_x,end_y\n"
        "# Fill measured_distance_m before data collection. Fill x/y after plotting trajectory.\n"
        "# corridor_length,30.50,0.0,0.0,0.0,0.0\n"
        "# room_width,6.00,0.0,0.0,0.0,0.0\n",
        encoding="utf-8",
    )


def command_block(args: argparse.Namespace, session_dir: Path) -> str:
    bag_root = Path.home() / "go1_ws" / "bags" / session_dir.name
    traj_dir = session_dir / "trajectories"
    logs_dir = session_dir / "logs"

    if args.kind.startswith("slam"):
        return f"""# 1. Record field bag (one terminal, real robot online)
ros2 launch go1_bringup go1_base.launch.py \\
  use_odom_fusion:=false \\
  bag_dir:={bag_root}

# 2. Baseline replay
# Terminal 1: algorithm
ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=false

# Terminal 2: trajectory recorder
ros2 run go1_bringup record_trajectory --ros-args \\
  -p use_sim_time:=true \\
  -p output_dir:={traj_dir} \\
  -p record_mode:=fastlio \\
  -p experiment_label:=baseline

# Terminal 3: bag playback
BAG=$(ls -td {bag_root}/*/ | head -1)
ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu /odom /imu

# 3. Degradation-aware replay
# Terminal 1: algorithm + log
ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true \\
  2>&1 | tee {logs_dir}/fastlio_improved.log

# Terminal 2: trajectory recorder
ros2 run go1_bringup record_trajectory --ros-args \\
  -p use_sim_time:=true \\
  -p output_dir:={traj_dir} \\
  -p record_mode:=fastlio \\
  -p experiment_label:=improved

# Terminal 3: same bag playback
BAG=$(ls -td {bag_root}/*/ | head -1)
ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu /odom /imu

# 4. Always-on high-weight replay
# Terminal 1: algorithm + log
ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true force_degraded:=true \\
  2>&1 | tee {logs_dir}/fastlio_always.log

# Terminal 2: trajectory recorder
ros2 run go1_bringup record_trajectory --ros-args \\
  -p use_sim_time:=true \\
  -p output_dir:={traj_dir} \\
  -p record_mode:=fastlio \\
  -p experiment_label:=always

# Terminal 3: same bag playback
BAG=$(ls -td {bag_root}/*/ | head -1)
ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu /odom /imu

# 5. Evaluation (after all three replays)
cp {session_dir}/geometry_constraints.csv {traj_dir}/geometry_constraints.csv
cp {logs_dir}/fastlio_improved.log {traj_dir}/fastlio_improved.log
cp {logs_dir}/fastlio_always.log {traj_dir}/fastlio_always.log
ros2 run go1_bringup evaluate_slam.sh {traj_dir}
"""

    if args.kind == "mapping":
        return f"""ros2 launch go1_bringup go1_mapping.launch.py \\
  use_odom_fusion:=false \\
  archive_label:={safe_name(args.label) or session_dir.name} \\
  bag_dir:={bag_root}
"""

    if args.kind.startswith("nav"):
        improved_args = ""
        if args.kind == "nav_improved":
            improved_args = " \\\n  use_odom_fusion:=true \\\n  enable_odom_constraint:=true"
        return f"""ros2 launch go1_bringup go1_nav.launch.py{improved_args} \\
  record_bag:=true \\
  bag_dir:={bag_root}

ros2 run go1_bringup nav_metrics --ros-args \\
  -p output_file:={session_dir}/nav_metrics.csv \\
  -p cmd_vel_output_file:={session_dir}/nav_metrics_cmd_vel.csv
"""

    return "# Add custom commands here.\n"


def write_commands(path: Path, args: argparse.Namespace, session_dir: Path) -> None:
    text = [
        "# Experiment Commands",
        "",
        "Run these commands after sourcing the workspace:",
        "",
        "```bash",
        "source ~/go1_ws/install/setup.bash",
        command_block(args, session_dir).rstrip(),
        "```",
        "",
    ]
    path.write_text("\n".join(text), encoding="utf-8")


def init_session(args: argparse.Namespace) -> int:
    root = Path(args.root).expanduser()
    label = safe_name(args.label)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    parts = [stamp, args.kind]
    if label:
        parts.append(label)
    session_dir = root / "_".join(parts)
    for subdir in ("bags", "trajectories", "logs", "screenshots", "results"):
        (session_dir / subdir).mkdir(parents=True, exist_ok=True)

    write_metadata(session_dir / "metadata.yaml", args, session_dir.name)
    write_geometry_template(session_dir / "geometry_constraints.csv")
    write_commands(session_dir / "COMMANDS.md", args, session_dir)
    print(session_dir)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    init = sub.add_parser("init", help="create an experiment session directory")
    init.add_argument("--root", default=str(Path.home() / "go1_ws" / "experiments"))
    init.add_argument("--kind", choices=KINDS, required=True)
    init.add_argument("--label", default="")
    init.add_argument("--operator", default="")
    init.add_argument("--location", default="")
    init.add_argument("--route", default="")
    init.add_argument("--notes", default="")
    init.set_defaults(func=init_session)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
