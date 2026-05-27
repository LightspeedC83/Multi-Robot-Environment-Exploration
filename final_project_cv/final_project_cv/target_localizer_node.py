import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros import TransformListener


def rotate_vector_by_quaternion(vector, quaternion):
    # tf2_geometry_msgs is not always installed in the course Docker image, so we
    # keep the point transform explicit instead of adding another dependency.
    x, y, z = vector
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)

    return rx, ry, rz


def transform_point(point, transform):
    rx, ry, rz = rotate_vector_by_quaternion(
        (point.point.x, point.point.y, point.point.z),
        transform.transform.rotation,
    )

    out = PointStamped()
    out.header.stamp = point.header.stamp
    out.header.frame_id = transform.header.frame_id
    out.point.x = rx + transform.transform.translation.x
    out.point.y = ry + transform.transform.translation.y
    out.point.z = rz + transform.transform.translation.z
    return out


class TargetLocalizer(Node):
    def __init__(self):
        super().__init__("target_localizer")

        self.declare_parameter("centroid_topic", "/target_centroid")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("object_diameter_m", 0.07)
        self.declare_parameter("assumed_depth_m", 1.0)
        self.declare_parameter("min_pixel_diameter", 3.0)
        self.declare_parameter("max_range_m", 10.0)
        self.declare_parameter("log_throttle_sec", 1.0)

        self.centroid_topic = self.get_parameter("centroid_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame_parameter = self.get_parameter("camera_frame").value
        self.object_diameter_m = float(self.get_parameter("object_diameter_m").value)
        self.assumed_depth_m = float(self.get_parameter("assumed_depth_m").value)
        self.min_pixel_diameter = float(self.get_parameter("min_pixel_diameter").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.log_throttle_sec = float(self.get_parameter("log_throttle_sec").value)

        self.camera_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.centroid_subscriber = self.create_subscription(
            PointStamped,
            self.centroid_topic,
            self.centroid_callback,
            10,
        )

        self.point_publisher = self.create_publisher(PointStamped, "/target_point_odom", 10)
        self.pose_publisher = self.create_publisher(PoseStamped, "/target_pose_odom", 10)

        self.get_logger().info(
            "Target localizer started. Expected centroid message: "
            "PointStamped point.x=u_px, point.y=v_px, point.z=observed_diameter_px."
        )

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def centroid_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn(
                "Waiting for CameraInfo before localizing target.",
                throttle_duration_sec=1.0,
            )
            return

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn("CameraInfo has invalid focal lengths.")
            return

        u = float(msg.point.x)
        v = float(msg.point.y)
        pixel_diameter = float(msg.point.z)

        depth = self.estimate_depth(pixel_diameter, fx, fy)
        if depth is None:
            return

        # Back-project the image centroid into the camera frame using the pinhole
        # camera model. Depth comes from the known-size target assumption.
        point_camera = PointStamped()
        point_camera.header.stamp = msg.header.stamp
        point_camera.header.frame_id = self.resolve_camera_frame(msg)
        point_camera.point.x = (u - cx) * depth / fx
        point_camera.point.y = (v - cy) * depth / fy
        point_camera.point.z = depth

        point_odom = self.to_target_frame(point_camera)
        if point_odom is None:
            return

        pose_odom = PoseStamped()
        pose_odom.header = point_odom.header
        pose_odom.pose.position = point_odom.point
        pose_odom.pose.orientation.w = 1.0

        self.point_publisher.publish(point_odom)
        self.pose_publisher.publish(pose_odom)

        self.get_logger().info(
            f"target in {self.target_frame}: "
            f"x={point_odom.point.x:.2f}, "
            f"y={point_odom.point.y:.2f}, "
            f"z={point_odom.point.z:.2f}, "
            f"range={depth:.2f} m",
            throttle_duration_sec=self.log_throttle_sec,
        )

    def estimate_depth(self, pixel_diameter, fx, fy):
        if pixel_diameter >= self.min_pixel_diameter:
            focal = 0.5 * (fx + fy)
            # Monocular size cue: Z = f * D / d_px.
            depth = focal * self.object_diameter_m / pixel_diameter
        else:
            depth = self.assumed_depth_m
            self.get_logger().warn(
                "Centroid message did not include a usable pixel diameter; "
                f"falling back to assumed_depth_m={depth:.2f}.",
                throttle_duration_sec=1.0,
            )

        if not math.isfinite(depth) or depth <= 0.0 or depth > self.max_range_m:
            self.get_logger().warn(f"Rejected invalid estimated depth: {depth:.2f}")
            return None

        return depth

    def resolve_camera_frame(self, centroid_msg):
        if self.camera_frame_parameter:
            return self.camera_frame_parameter

        if centroid_msg.header.frame_id:
            return centroid_msg.header.frame_id

        return self.camera_info.header.frame_id

    def to_target_frame(self, point_camera):
        if point_camera.header.frame_id == self.target_frame:
            return point_camera

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point_camera.header.frame_id,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"Could not transform {point_camera.header.frame_id} to "
                f"{self.target_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

        return transform_point(point_camera, transform)


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizer()
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
