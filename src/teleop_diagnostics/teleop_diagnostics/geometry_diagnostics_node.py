# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Optional observer-only ROS node wrapper (no control / safety / recorder)."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node

from teleop_diagnostics.geometry_cli import run_report


class GeometryDiagnosticsNode(Node):
    """Runs a one-shot offline report; does not publish commands."""

    def __init__(self):
        super().__init__("geometry_diagnostics")
        self.declare_parameter("out_dir", str(Path.cwd() / "evidence" / "geometry_stage1"))
        self.declare_parameter("random_count", 5)
        self.declare_parameter("seed", 20260813)
        out_dir = Path(str(self.get_parameter("out_dir").value))
        random_count = int(self.get_parameter("random_count").value)
        seed = int(self.get_parameter("seed").value)
        manifest = run_report(out_dir, random_count=random_count, seed=seed)
        self.get_logger().info(
            f"Stage-1 geometry report written to {out_dir} "
            f"(commit={manifest.get('commit')}, physical=NOT_RUN/UNAVAILABLE)"
        )


def main(argv=None):
    rclpy.init(args=argv)
    node = GeometryDiagnosticsNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
