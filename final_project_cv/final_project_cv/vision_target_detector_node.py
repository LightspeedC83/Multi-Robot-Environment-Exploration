import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


def bbox_area(bbox):
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def bbox_center(bbox):
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def bbox_diameter(bbox):
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1, y2 - y1)


def bbox_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    intersection = bbox_area([ix1, iy1, ix2, iy2])
    union = bbox_area(a) + bbox_area(b) - intersection

    if union <= 0.0:
        return 0.0

    return intersection / union


def mask_measurement(mask):
    ys, xs = np.where(mask > 0.5)

    if len(xs) == 0:
        return None

    # Use the segmented region, not the YOLO box, when estimating where the
    # object center and apparent diameter are in the image.
    centroid = float(np.mean(xs)), float(np.mean(ys))
    pixel_diameter = max(float(np.max(xs) - np.min(xs) + 1), float(np.max(ys) - np.min(ys) + 1))
    return centroid, pixel_diameter


def resize_mask_to_frame(mask, frame_shape):
    frame_height, frame_width = frame_shape[:2]

    if mask.shape[:2] == (frame_height, frame_width):
        return mask

    return cv2.resize(
        mask.astype(np.float32),
        (frame_width, frame_height),
        interpolation=cv2.INTER_NEAREST,
    )


def smooth_values(previous, current, alpha):
    if previous is None or alpha <= 0.0:
        return current

    return [
        alpha * previous_value + (1.0 - alpha) * current_value
        for previous_value, current_value in zip(previous, current)
    ]


def smooth_value(previous, current, alpha):
    if previous is None or alpha <= 0.0:
        return current

    return alpha * previous + (1.0 - alpha) * current


def resize_to_width(frame, max_width):
    if max_width is None or max_width <= 0:
        return frame, 1.0

    height, width = frame.shape[:2]
    if width <= max_width:
        return frame, 1.0

    scale = max_width / float(width)
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(frame, (max_width, resized_height), interpolation=cv2.INTER_AREA)
    return resized, scale


def parameter_as_bool(value):
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}

    return bool(value)


def resolve_model_path(model_name):
    path = Path(model_name)
    if path.exists():
        return str(path)

    try:
        package_share = Path(get_package_share_directory("final_project_cv"))
    except PackageNotFoundError:
        return model_name

    candidates = [
        package_share / "models" / model_name,
        package_share / model_name,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return model_name


def load_yolo_model(weights_path):
    # Ultralytics imports are delayed so the logic-only ROS nodes can run
    # without loading PyTorch.
    from ultralytics import YOLO

    return YOLO(weights_path)


def load_fastsam_model(weights_path):
    from ultralytics import FastSAM

    return FastSAM(weights_path)


class VisionTargetDetector(Node):
    def __init__(self):
        super().__init__("vision_target_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("centroid_topic", "/target_centroid")
        self.declare_parameter("debug_image_topic", "/target_debug_image")
        self.declare_parameter("target", "bottle")
        self.declare_parameter("yolo_weights", "yolo11n.pt")
        self.declare_parameter("fastsam_weights", "FastSAM-s.pt")
        self.declare_parameter("use_fastsam", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("yolo_conf", 0.25)
        self.declare_parameter("fastsam_conf", 0.4)
        self.declare_parameter("fastsam_iou", 0.9)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("process_width", 640)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("smooth_alpha", 0.65)
        self.declare_parameter("selection_strategy", "largest")
        self.declare_parameter("torch_threads", 2)
        self.declare_parameter("fuse_yolo_model", True)
        self.declare_parameter("disable_nnpack", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.centroid_topic = self.get_parameter("centroid_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.target = self.get_parameter("target").value
        self.yolo_weights = resolve_model_path(self.get_parameter("yolo_weights").value)
        self.fastsam_weights = resolve_model_path(self.get_parameter("fastsam_weights").value)
        self.use_fastsam = parameter_as_bool(self.get_parameter("use_fastsam").value)
        self.publish_debug_image = parameter_as_bool(self.get_parameter("publish_debug_image").value)
        self.yolo_conf = float(self.get_parameter("yolo_conf").value)
        self.fastsam_conf = float(self.get_parameter("fastsam_conf").value)
        self.fastsam_iou = float(self.get_parameter("fastsam_iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.process_width = int(self.get_parameter("process_width").value)
        self.process_every_n = int(self.get_parameter("process_every_n").value)
        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)
        self.selection_strategy = self.get_parameter("selection_strategy").value
        self.torch_threads = int(self.get_parameter("torch_threads").value)
        self.fuse_yolo_model = parameter_as_bool(self.get_parameter("fuse_yolo_model").value)
        self.disable_nnpack = parameter_as_bool(self.get_parameter("disable_nnpack").value)

        if self.process_every_n < 1:
            raise ValueError("process_every_n must be >= 1")

        if not 0.0 <= self.smooth_alpha < 1.0:
            raise ValueError("smooth_alpha must be >= 0.0 and < 1.0")

        self.bridge = CvBridge()
        self.configure_torch_threads()
        self.detector = load_yolo_model(self.yolo_weights)
        self.fuse_detector_if_requested()
        self.segmenter = load_fastsam_model(self.fastsam_weights) if self.use_fastsam else None
        self.frame_count = 0
        self.tracked_bbox = None
        self.tracked_centroid = None
        self.tracked_diameter = None

        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.centroid_publisher = self.create_publisher(PointStamped, self.centroid_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info(
            f"Vision detector started: target='{self.target}', "
            f"use_fastsam={self.use_fastsam}, image_topic='{self.image_topic}', "
            f"process_width={self.process_width}, imgsz={self.imgsz}, "
            f"process_every_n={self.process_every_n}."
        )

    def configure_torch_threads(self):
        if self.torch_threads <= 0:
            return

        try:
            import torch

            torch.set_num_threads(self.torch_threads)
            if self.disable_nnpack and hasattr(torch.backends, "nnpack"):
                torch.backends.nnpack.enabled = False
        except Exception as exc:
            self.get_logger().debug(f"Could not configure torch thread count: {exc}")

    def fuse_detector_if_requested(self):
        if not self.fuse_yolo_model:
            return

        try:
            self.detector.fuse()
        except Exception as exc:
            self.get_logger().debug(f"Could not fuse YOLO model: {exc}")

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        processed_frame, scale = resize_to_width(frame, self.process_width)

        yolo_results = self.detector(
            processed_frame,
            conf=self.yolo_conf,
            imgsz=self.imgsz,
            verbose=False,
        )
        detections = self.get_detections_by_class(yolo_results[0])
        selected = self.select_detection(detections, processed_frame.shape)

        if selected is None:
            self.publish_debug_frame(msg, processed_frame)
            return

        # YOLO answers "which target and where roughly?" FastSAM, when enabled,
        # refines that into a mask-based centroid for localization.
        bbox, confidence, class_id = selected
        bbox = smooth_values(self.tracked_bbox, bbox, self.smooth_alpha)
        self.tracked_bbox = bbox

        measurement = self.compute_measurement(processed_frame, bbox)
        if measurement is None:
            self.publish_debug_frame(msg, processed_frame)
            return

        centroid, pixel_diameter, mask = measurement
        centroid = tuple(smooth_values(self.tracked_centroid, centroid, self.smooth_alpha))
        self.tracked_centroid = centroid
        pixel_diameter = smooth_value(self.tracked_diameter, pixel_diameter, self.smooth_alpha)
        self.tracked_diameter = pixel_diameter

        self.publish_centroid(msg, centroid, pixel_diameter, scale)
        self.publish_debug_frame(msg, processed_frame, bbox, centroid, confidence, mask)

    def get_detections_by_class(self, result):
        if result.boxes is None or len(result.boxes) == 0:
            return []

        detections = []
        names = result.names

        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = names[class_id]
            confidence = float(box.conf[0])

            if class_name != self.target:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            bbox = [float(x1), float(y1), float(x2), float(y2)]
            detections.append((bbox, confidence, class_id))

        return detections

    def select_detection(self, detections, frame_shape):
        if not detections:
            return None

        if self.tracked_bbox is not None:
            return max(detections, key=lambda item: (bbox_iou(item[0], self.tracked_bbox), item[1]))

        height, width = frame_shape[:2]
        frame_center = (width / 2.0, height / 2.0)

        if self.selection_strategy == "confidence":
            return max(detections, key=lambda item: item[1])

        if self.selection_strategy == "largest":
            return max(detections, key=lambda item: (bbox_area(item[0]), item[1]))

        if self.selection_strategy == "bottom":
            return max(detections, key=lambda item: (item[0][3], bbox_area(item[0]), item[1]))

        if self.selection_strategy == "center":
            def center_score(item):
                cx, cy = bbox_center(item[0])
                distance = math.hypot(cx - frame_center[0], cy - frame_center[1])
                return (-distance, item[1])

            return max(detections, key=center_score)

        raise ValueError(f"Unknown selection_strategy: {self.selection_strategy}")

    def compute_measurement(self, frame, bbox):
        fallback = (bbox_center(bbox), bbox_diameter(bbox), None)

        if not self.use_fastsam:
            return fallback

        sam_results = self.segmenter(
            frame,
            bboxes=[bbox],
            imgsz=self.imgsz,
            retina_masks=True,
            conf=self.fastsam_conf,
            iou=self.fastsam_iou,
            verbose=False,
        )

        if not sam_results or sam_results[0].masks is None:
            return fallback

        mask = sam_results[0].masks.data[0].cpu().numpy()
        mask = resize_mask_to_frame(mask, frame.shape)
        measurement = mask_measurement(mask)

        if measurement is None:
            return fallback

        centroid, pixel_diameter = measurement
        return centroid, pixel_diameter, mask

    def publish_centroid(self, image_msg, centroid, pixel_diameter, scale):
        u, v = centroid

        if scale != 0.0:
            u /= scale
            v /= scale
            pixel_diameter /= scale

        msg = PointStamped()
        msg.header = image_msg.header
        msg.point.x = float(u)
        msg.point.y = float(v)
        msg.point.z = float(pixel_diameter)
        self.centroid_publisher.publish(msg)

        self.get_logger().info(
            f"target={self.target}, u={u:.1f}, v={v:.1f}, "
            f"diameter_px={pixel_diameter:.1f}",
            throttle_duration_sec=0.5,
        )

    def publish_debug_frame(self, image_msg, frame, bbox=None, centroid=None, confidence=None, mask=None):
        if not self.publish_debug_image:
            return

        debug = frame.copy()

        if mask is not None:
            mask = resize_mask_to_frame(mask, debug.shape) > 0.5
            overlay = debug.copy()
            overlay[mask] = (0, 180, 80)
            debug = cv2.addWeighted(overlay, 0.35, debug, 0.65, 0.0)

        if bbox is not None:
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 0, 0), 2)

        if centroid is not None:
            u, v = centroid
            cv2.circle(debug, (int(u), int(v)), 5, (0, 0, 255), -1)

        if confidence is not None:
            cv2.putText(
                debug,
                f"{self.target} conf={confidence:.2f}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_image_publisher.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionTargetDetector()
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
