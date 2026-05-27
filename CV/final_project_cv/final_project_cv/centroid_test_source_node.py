import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CentroidTestSource(Node):
    def __init__(self):
        super().__init__("centroid_test_source")

        self.declare_parameter("centroid_topic", "/target_centroid")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 554.0)
        self.declare_parameter("fy", 554.0)
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("center_u", 320.0)
        self.declare_parameter("center_v", 240.0)
        self.declare_parameter("u_amplitude", 120.0)
        self.declare_parameter("v_amplitude", 45.0)
        self.declare_parameter("diameter_px", 90.0)
        self.declare_parameter("diameter_amplitude_px", 20.0)
        self.declare_parameter("log_throttle_sec", 1.0)

        self.centroid_topic = self.get_parameter("centroid_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        fps = float(self.get_parameter("fps").value)
        self.center_u = float(self.get_parameter("center_u").value)
        self.center_v = float(self.get_parameter("center_v").value)
        self.u_amplitude = float(self.get_parameter("u_amplitude").value)
        self.v_amplitude = float(self.get_parameter("v_amplitude").value)
        self.diameter_px = float(self.get_parameter("diameter_px").value)
        self.diameter_amplitude_px = float(self.get_parameter("diameter_amplitude_px").value)
        self.log_throttle_sec = float(self.get_parameter("log_throttle_sec").value)

        self.t = 0.0
        self.dt = 1.0 / max(fps, 0.1)
        self.camera_info = self.build_camera_info()
        self.centroid_publisher = self.create_publisher(PointStamped, self.centroid_topic, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.timer = self.create_timer(self.dt, self.publish_measurement)

        self.get_logger().info(
            "Centroid test source started. Publishing synthetic centroid measurements "
            f"in frame '{self.frame_id}'."
        )

    def build_camera_info(self):
        cx = self.width / 2.0
        cy = self.height / 2.0

        # This camera model is synthetic but internally consistent. It lets us
        # test the localization math without needing the robot camera online.
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [self.fx, 0.0, cx, 0.0, self.fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [self.fx, 0.0, cx, 0.0, 0.0, self.fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_measurement(self):
        stamp = self.get_clock().now().to_msg()

        # The varying centroid imitates what a perception node would publish as
        # the target moves through the image and changes apparent size.
        u = self.center_u + self.u_amplitude * math.sin(self.t)
        v = self.center_v + self.v_amplitude * math.sin(0.7 * self.t)
        diameter = self.diameter_px + self.diameter_amplitude_px * math.sin(0.45 * self.t)
        diameter = max(3.0, diameter)

        camera_info = self.camera_info
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self.frame_id
        self.camera_info_publisher.publish(camera_info)

        centroid = PointStamped()
        centroid.header.stamp = stamp
        centroid.header.frame_id = self.frame_id
        centroid.point.x = u
        centroid.point.y = v
        centroid.point.z = diameter
        self.centroid_publisher.publish(centroid)

        self.get_logger().info(
            f"centroid u={u:.1f}, v={v:.1f}, diameter_px={diameter:.1f}",
            throttle_duration_sec=self.log_throttle_sec,
        )

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = CentroidTestSource()
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
