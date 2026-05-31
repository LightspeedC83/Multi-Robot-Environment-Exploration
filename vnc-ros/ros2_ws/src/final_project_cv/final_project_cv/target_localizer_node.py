import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros import TransformListener


def project_detection_message(class_name):
    if class_name == "sports ball":
        return "goal found sphere detected"

    if class_name == "bottle":
        return "heuristics detected"

    return class_name


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
        self.declare_parameter("scan_topic", "")
        self.declare_parameter("point_topic", "/target_point_odom")
        self.declare_parameter("pose_topic", "/target_pose_odom")
        self.declare_parameter("target", "bottle")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("lidar_frame", "")
        self.declare_parameter("object_diameter_m", 0.07)
        self.declare_parameter("assumed_depth_m", 1.0)
        self.declare_parameter("min_pixel_diameter", 3.0)
        self.declare_parameter("max_range_m", 10.0)
        self.declare_parameter("lidar_window_deg", 2.0)
        self.declare_parameter("log_throttle_sec", 1.0)

        self.centroid_topic = self.get_parameter("centroid_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.point_topic = self.get_parameter("point_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.target = self.get_parameter("target").value
        self.event_label = project_detection_message(self.target)
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame_parameter = self.get_parameter("camera_frame").value
        self.lidar_frame_parameter = self.get_parameter("lidar_frame").value
        self.object_diameter_m = float(self.get_parameter("object_diameter_m").value)
        self.assumed_depth_m = float(self.get_parameter("assumed_depth_m").value)
        self.min_pixel_diameter = float(self.get_parameter("min_pixel_diameter").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.lidar_window_deg = float(self.get_parameter("lidar_window_deg").value)
        self.log_throttle_sec = float(self.get_parameter("log_throttle_sec").value)

        self.camera_info = None
        self.latest_scan = None

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
        if self.scan_topic:
            self.scan_subscriber = self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.scan_callback,
                10,
            )
        else:
            self.scan_subscriber = None

        self.point_publisher = self.create_publisher(PointStamped, self.point_topic, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.get_logger().debug(
            "Target localizer ready for PointStamped centroid messages."
        )

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

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

        lidar_text = self.describe_lidar_measurement(point_odom)

        self.get_logger().info(
            f"{self.event_label}: "
            f"{self.target_frame}=("
            f"x={point_odom.point.x:.2f}, "
            f"y={point_odom.point.y:.2f}, "
            f"z={point_odom.point.z:.2f}), "
            f"range={depth:.2f} m, "
            f"image_centroid=({u:.1f},{v:.1f}) px, "
            f"{lidar_text}",
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

    def describe_lidar_measurement(self, point_target_frame):
        if self.latest_scan is None:
            return "lidar=no scan yet"

        scan_frame = self.lidar_frame_parameter or self.latest_scan.header.frame_id
        if not scan_frame:
            return "lidar=no scan frame"

        point_scan = self.to_frame(point_target_frame, scan_frame)
        if point_scan is None:
            return f"lidar=missing TF to {scan_frame}"

        # LiDAR gives us a quick sanity check beside the vision-only range
        x = float(point_scan.point.x)
        y = float(point_scan.point.y)
        expected_range = math.hypot(x, y)
        bearing = math.atan2(y, x)

        beam_range = self.pick_lidar_beam(self.latest_scan, bearing, expected_range)
        bearing_deg = math.degrees(bearing)
        if beam_range is None:
            return (
                f"lidar=no valid beam near {bearing_deg:.1f} deg, "
                f"vision_planar_range={expected_range:.2f} m"
            )

        return (
            f"lidar={beam_range:.2f} m near {bearing_deg:.1f} deg, "
            f"vision_planar_range={expected_range:.2f} m"
        )

    def pick_lidar_beam(self, scan, bearing, expected_range):
        if scan.angle_increment == 0.0 or not scan.ranges:
            return None

        center = round((bearing - scan.angle_min) / scan.angle_increment)
        center = int(center)
        if center < 0 or center >= len(scan.ranges):
            return None

        #small window because the Gazebo scan is coarse at 180 beams
        window = max(
            0,
            int(round(math.radians(self.lidar_window_deg) / abs(scan.angle_increment))),
        )
        lo = max(0, center - window)
        hi = min(len(scan.ranges), center + window + 1)

        candidates = []
        for value in scan.ranges[lo:hi]:
            if (
                math.isfinite(value)
                and value >= scan.range_min
                and value <= scan.range_max
            ):
                candidates.append(float(value))

        if not candidates:
            return None

        # this picks the beam most consistent with the CV estimate, not just the closest wall
        return min(candidates, key=lambda value: abs(value - expected_range))

    def to_frame(self, point, frame_id):
        if point.header.frame_id == frame_id:
            return point

        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                point.header.frame_id,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        return transform_point(point, transform)


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
