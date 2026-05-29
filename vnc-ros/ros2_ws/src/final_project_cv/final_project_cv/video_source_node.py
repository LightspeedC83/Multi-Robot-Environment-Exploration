import math
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def parameter_as_bool(value):
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}

    return bool(value)


class VideoSource(Node):
    def __init__(self):
        super().__init__("video_source")

        self.declare_parameter("source", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("horizontal_fov_deg", 60.0)

        self.source = self.get_parameter("source").value
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.fps = float(self.get_parameter("fps").value)
        self.loop = parameter_as_bool(self.get_parameter("loop").value)
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)

        if not self.source:
            raise ValueError("video_source requires source:=/path/to/video.mp4")

        source_path = Path(self.source).expanduser()
        if not source_path.exists():
            raise FileNotFoundError(f"Video source not found: {source_path}")

        self.capture = cv2.VideoCapture(str(source_path))
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open video source: {source_path}")

        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        source_fps = float(self.capture.get(cv2.CAP_PROP_FPS))

        if self.fps <= 0.0:
            self.fps = source_fps if source_fps > 0.0 else 10.0

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.camera_info = self.build_camera_info()
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)

        self.get_logger().info(
            f"Video source started: source='{source_path}', size={self.width}x{self.height}, "
            f"fps={self.fps:.1f}, frame_id='{self.frame_id}'."
        )

    def build_camera_info(self):
        fov_rad = math.radians(self.horizontal_fov_deg)
        fx = self.width / (2.0 * math.tan(fov_rad / 2.0))
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_frame(self):
        ok, frame = self.capture.read()

        if not ok:
            if not self.loop:
                self.get_logger().info("Video ended; stopping publisher.")
                self.timer.cancel()
                return

            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
            if not ok:
                self.get_logger().warn("Could not read frame after rewinding video.")
                return

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        camera_info_msg = self.camera_info
        camera_info_msg.header.stamp = stamp
        camera_info_msg.header.frame_id = self.frame_id

        self.image_publisher.publish(image_msg)
        self.camera_info_publisher.publish(camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoSource()
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
