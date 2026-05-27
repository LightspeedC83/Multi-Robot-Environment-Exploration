import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class TargetTraceRecorder(Node):
    def __init__(self):
        super().__init__("target_trace_recorder")

        self.declare_parameter("centroid_topic", "/target_centroid")
        self.declare_parameter("target_point_topic", "/target_point_odom")
        self.declare_parameter("output_dir", "/root/ros2_ws/src/final_project_cv/output")
        self.declare_parameter("save_plot_every_sec", 2.0)
        self.declare_parameter("max_samples", 1000)

        centroid_topic = self.get_parameter("centroid_topic").value
        target_point_topic = self.get_parameter("target_point_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.save_plot_every_sec = float(self.get_parameter("save_plot_every_sec").value)
        self.max_samples = int(self.get_parameter("max_samples").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / "target_trace.csv"
        self.plot_path = self.output_dir / "target_trace.png"

        self.latest_centroid = None
        self.rows = []
        self.write_csv_header()

        self.create_subscription(PointStamped, centroid_topic, self.centroid_callback, 10)
        self.create_subscription(PointStamped, target_point_topic, self.target_point_callback, 10)
        self.create_timer(self.save_plot_every_sec, self.save_plot)

        self.get_logger().info(
            f"Trace recorder started. CSV: {self.csv_path}; plot: {self.plot_path}"
        )

    def write_csv_header(self):
        with self.csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "sample",
                "time_sec",
                "u_px",
                "v_px",
                "diameter_px",
                "odom_x_m",
                "odom_y_m",
                "odom_z_m",
                "range_m",
            ])

    def centroid_callback(self, msg):
        self.latest_centroid = msg

    def target_point_callback(self, msg):
        if self.latest_centroid is None:
            return

        # Pair the most recent image-space measurement with the current odom
        # estimate so the CSV tells the whole story for each sample.
        point = msg.point
        centroid = self.latest_centroid.point
        stamp = msg.header.stamp
        time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        range_m = math.sqrt(point.x ** 2 + point.y ** 2 + point.z ** 2)

        row = {
            "sample": len(self.rows),
            "time_sec": time_sec,
            "u_px": float(centroid.x),
            "v_px": float(centroid.y),
            "diameter_px": float(centroid.z),
            "odom_x_m": float(point.x),
            "odom_y_m": float(point.y),
            "odom_z_m": float(point.z),
            "range_m": range_m,
        }

        self.rows.append(row)
        if len(self.rows) > self.max_samples:
            self.rows = self.rows[-self.max_samples:]

        with self.csv_path.open("a", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=row.keys())
            writer.writerow(row)

    def save_plot(self):
        if len(self.rows) < 2:
            return

        try:
            # Use the non-interactive backend because this often runs inside a
            # terminal-only Docker container.
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().warn(f"Could not import matplotlib for plotting: {exc}")
            return

        samples = [row["sample"] for row in self.rows]
        u_values = [row["u_px"] for row in self.rows]
        v_values = [row["v_px"] for row in self.rows]
        x_values = [row["odom_x_m"] for row in self.rows]
        y_values = [row["odom_y_m"] for row in self.rows]
        z_values = [row["odom_z_m"] for row in self.rows]
        range_values = [row["range_m"] for row in self.rows]

        fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)

        axes[0].plot(u_values, v_values, marker="o", linewidth=1.5, markersize=3)
        axes[0].invert_yaxis()
        axes[0].set_title("Centroid Path")
        axes[0].set_xlabel("u (px)")
        axes[0].set_ylabel("v (px)")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(z_values, x_values, marker="o", linewidth=1.5, markersize=3)
        axes[1].set_title("Odom Top View")
        axes[1].set_xlabel("z / depth (m)")
        axes[1].set_ylabel("x / lateral (m)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(samples, range_values, label="range (m)", linewidth=1.5)
        axes[2].plot(samples, y_values, label="y height (m)", linewidth=1.5)
        axes[2].set_title("Target Over Time")
        axes[2].set_xlabel("sample")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        fig.savefig(self.plot_path, dpi=150)
        plt.close(fig)

        self.get_logger().info(
            f"recorded {len(self.rows)} samples -> {self.csv_path}, {self.plot_path}",
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetTraceRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
