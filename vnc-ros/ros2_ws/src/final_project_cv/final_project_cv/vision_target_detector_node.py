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


def parse_class_list(value):
    if isinstance(value, (list, tuple)):
        return {str(item).strip() for item in value if str(item).strip()}

    return {
        item.strip()
        for item in str(value).split(",")
        if item.strip()
    }


def project_detection_message(class_name):
    if class_name == "sports ball":
        return "goal found sphere detected"

    if class_name == "bottle":
        return "heuristics detected"

    return class_name


def compact_detection_label(class_name):
    if class_name == "sports ball":
        return "goal sphere"

    if class_name == "bottle":
        return "heuristic"

    return class_name


def detection_bbox_from_mask(mask):
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None

    return [
        float(np.min(xs)),
        float(np.min(ys)),
        float(np.max(xs) + 1),
        float(np.max(ys) + 1),
    ]


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
        self.declare_parameter("use_yolo", True)
        self.declare_parameter("yolo_weights", "yolo11n.pt")
        self.declare_parameter("fastsam_weights", "FastSAM-s.pt")
        self.declare_parameter("use_fastsam", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("draw_all_detections", True)
        self.declare_parameter("display_classes", "bottle,sports ball")
        self.declare_parameter("use_sim_bottle_color_fallback", True)
        self.declare_parameter("sim_bottle_min_area", 25.0)
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
        self.use_yolo = parameter_as_bool(self.get_parameter("use_yolo").value)
        self.yolo_weights = resolve_model_path(self.get_parameter("yolo_weights").value)
        self.fastsam_weights = resolve_model_path(self.get_parameter("fastsam_weights").value)
        self.use_fastsam = parameter_as_bool(self.get_parameter("use_fastsam").value)
        self.publish_debug_image = parameter_as_bool(self.get_parameter("publish_debug_image").value)
        self.draw_all_detections = parameter_as_bool(self.get_parameter("draw_all_detections").value)
        self.display_classes = parse_class_list(self.get_parameter("display_classes").value)
        self.use_sim_bottle_color_fallback = parameter_as_bool(
            self.get_parameter("use_sim_bottle_color_fallback").value
        )
        self.sim_bottle_min_area = float(self.get_parameter("sim_bottle_min_area").value)
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
        self.detector = load_yolo_model(self.yolo_weights) if self.use_yolo else None
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

        self.get_logger().debug(
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
        if self.detector is None or not self.fuse_yolo_model:
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

        all_detections = []
        if self.detector is not None:
            yolo_results = self.detector(
                processed_frame,
                conf=self.yolo_conf,
                imgsz=self.imgsz,
                verbose=False,
            )
            all_detections.extend(self.get_all_detections(yolo_results[0]))
        if self.use_sim_bottle_color_fallback:
            all_detections.extend(self.detect_sim_color_targets(processed_frame))

        visible_detections = [
            detection for detection in all_detections
            if detection["class_name"] in self.display_classes
        ]
        detections = [
            detection for detection in visible_detections
            if detection["class_name"] == self.target
        ]
        selected = self.select_detection(detections, processed_frame.shape)

        if selected is None:
            if visible_detections:
                status_text = self.project_detection_summary(visible_detections)
            else:
                status_text = "no bottle/sports ball"
            self.publish_debug_frame(
                msg,
                processed_frame,
                all_detections=visible_detections,
                status_text=status_text,
            )
            return

        # YOLO answers "which target and where roughly?" FastSAM, when enabled,
        # refines that into a mask-based centroid for localization.
        bbox = selected["bbox"]
        confidence = selected["confidence"]
        bbox = smooth_values(self.tracked_bbox, bbox, self.smooth_alpha)
        self.tracked_bbox = bbox

        selected_for_measurement = dict(selected)
        selected_for_measurement["bbox"] = bbox
        measurement = self.compute_measurement(processed_frame, selected_for_measurement)
        if measurement is None:
            self.publish_debug_frame(
                msg,
                processed_frame,
                all_detections=visible_detections,
                status_text=f"no measurement for target='{self.target}'",
            )
            return

        centroid, pixel_diameter, mask, measurement_source = measurement
        centroid = tuple(smooth_values(self.tracked_centroid, centroid, self.smooth_alpha))
        self.tracked_centroid = centroid
        pixel_diameter = smooth_value(self.tracked_diameter, pixel_diameter, self.smooth_alpha)
        self.tracked_diameter = pixel_diameter

        self.publish_centroid(
            msg,
            centroid,
            pixel_diameter,
            scale,
            selected["class_name"],
            measurement_source,
        )
        self.publish_debug_frame(
            msg,
            processed_frame,
            bbox,
            centroid,
            confidence,
            mask,
            target_class=selected["class_name"],
            all_detections=visible_detections,
            measurement_source=measurement_source,
        )

    def project_detection_summary(self, detections):
        messages = []
        for class_name in ("sports ball", "bottle"):
            class_detections = [
                detection for detection in detections
                if detection["class_name"] == class_name
            ]
            if not class_detections:
                continue

            best = max(class_detections, key=lambda item: item["confidence"])
            messages.append(f"{project_detection_message(class_name)} ({best['confidence']:.2f})")

        return " | ".join(messages)

    def get_all_detections(self, result):
        if result.boxes is None or len(result.boxes) == 0:
            return []

        detections = []
        names = result.names

        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = names[class_id]
            confidence = float(box.conf[0])

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            bbox = [float(x1), float(y1), float(x2), float(y2)]
            detections.append({
                "bbox": bbox,
                "confidence": confidence,
                "class_id": class_id,
                "class_name": class_name,
                "source": "yolo",
                "mask": None,
            })

        return detections

    def detect_sim_color_targets(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # The lightweight Gazebo bottles are deliberately cyan/blue. This
        # fallback is only for the synthetic world where pretrained YOLO does
        # not see simple cylinder models as real bottles. The goal ball is
        # deliberately orange for the same reason.
        lower_blue = np.array([82, 45, 35], dtype=np.uint8)
        upper_blue = np.array([112, 255, 235], dtype=np.uint8)
        lower_orange = np.array([4, 70, 70], dtype=np.uint8)
        upper_orange = np.array([25, 255, 255], dtype=np.uint8)

        detections = []
        detections.extend(self.detect_sim_color_components(
            hsv,
            lower_blue,
            upper_blue,
            class_name="bottle",
            min_area=self.sim_bottle_min_area,
            require_tall=True,
            confidence=0.95,
        ))
        detections.extend(self.detect_sim_color_components(
            hsv,
            lower_orange,
            upper_orange,
            class_name="sports ball",
            min_area=12.0,
            require_tall=False,
            confidence=0.92,
        ))
        return detections

    def detect_sim_color_components(self, hsv, lower, upper, class_name, min_area, require_tall, confidence):
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h < 3 or w < 3:
                continue
            if require_tall and (h < 10 or h < 1.35 * w):
                continue

            component_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(component_mask, [contour], -1, 255, thickness=cv2.FILLED)
            bbox = detection_bbox_from_mask(component_mask)
            if bbox is None:
                continue

            detections.append({
                "bbox": bbox,
                "confidence": confidence,
                "class_id": -1,
                "class_name": class_name,
                "source": "sim_color",
                "mask": component_mask.astype(np.float32) / 255.0,
            })

        return detections

    def select_detection(self, detections, frame_shape):
        if not detections:
            return None

        if self.tracked_bbox is not None:
            return max(
                detections,
                key=lambda item: (bbox_iou(item["bbox"], self.tracked_bbox), item["confidence"]),
            )

        height, width = frame_shape[:2]
        frame_center = (width / 2.0, height / 2.0)

        if self.selection_strategy == "confidence":
            return max(detections, key=lambda item: item["confidence"])

        if self.selection_strategy == "largest":
            return max(detections, key=lambda item: (bbox_area(item["bbox"]), item["confidence"]))

        if self.selection_strategy == "bottom":
            return max(
                detections,
                key=lambda item: (item["bbox"][3], bbox_area(item["bbox"]), item["confidence"]),
            )

        if self.selection_strategy == "center":
            def center_score(item):
                cx, cy = bbox_center(item["bbox"])
                distance = math.hypot(cx - frame_center[0], cy - frame_center[1])
                return (-distance, item["confidence"])

            return max(detections, key=center_score)

        raise ValueError(f"Unknown selection_strategy: {self.selection_strategy}")

    def compute_measurement(self, frame, detection):
        bbox = detection["bbox"]
        detection_mask = detection.get("mask")
        detection_source = detection.get("source")

        fallback = (bbox_center(bbox), bbox_diameter(bbox), None, "bbox")

        if self.use_fastsam:
            sam_results = self.segmenter(
                frame,
                bboxes=[bbox],
                imgsz=self.imgsz,
                retina_masks=True,
                conf=self.fastsam_conf,
                iou=self.fastsam_iou,
                verbose=False,
            )

            if sam_results and sam_results[0].masks is not None:
                mask = sam_results[0].masks.data[0].cpu().numpy()
                mask = resize_mask_to_frame(mask, frame.shape)
                measurement = mask_measurement(mask)

                if measurement is not None:
                    centroid, pixel_diameter = measurement
                    return centroid, pixel_diameter, mask, "fastsam"

        if detection_mask is not None:
            measurement = mask_measurement(detection_mask)
            if measurement is not None:
                centroid, pixel_diameter = measurement
                return centroid, pixel_diameter, detection_mask, detection_source

        return fallback

    def publish_centroid(
        self,
        image_msg,
        centroid,
        pixel_diameter,
        scale,
        target_class,
        measurement_source,
    ):
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
            f"{project_detection_message(target_class)}: "
            f"centroid=({u:.1f},{v:.1f}) px, "
            f"diameter_px={pixel_diameter:.1f}, source={measurement_source}",
            throttle_duration_sec=0.5,
        )

    def publish_debug_frame(
        self,
        image_msg,
        frame,
        bbox=None,
        centroid=None,
        confidence=None,
        mask=None,
        target_class=None,
        all_detections=None,
        status_text=None,
        measurement_source=None,
    ):
        if not self.publish_debug_image:
            return

        debug = frame.copy()

        if self.draw_all_detections:
            for detection in all_detections or []:
                x1, y1, x2, y2 = map(int, detection["bbox"])
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 210, 255), 1)
                cv2.putText(
                    debug,
                    f"{compact_detection_label(detection['class_name'])} {detection['confidence']:.2f}",
                    (x1, max(15, y1 - 6)),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.32,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

        if mask is not None:
            mask = resize_mask_to_frame(mask, debug.shape) > 0.5
            overlay = debug.copy()
            overlay[mask] = (0, 180, 80)
            debug = cv2.addWeighted(overlay, 0.45, debug, 0.55, 0.0)
            contours, _ = cv2.findContours(
                mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(debug, contours, -1, (0, 120, 0), 1)

        if bbox is not None:
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 0, 0), 2)

        if centroid is not None:
            u, v = centroid
            center = (int(u), int(v))
            cv2.circle(debug, center, 5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.circle(debug, center, 3, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.drawMarker(
                debug,
                center,
                (0, 0, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=12,
                thickness=1,
                line_type=cv2.LINE_AA,
            )

        if confidence is not None:
            source = f" | {measurement_source}" if measurement_source else ""
            cv2.putText(
                debug,
                f"{project_detection_message(target_class or self.target)} {confidence:.2f}{source}",
                (8, 18),
                cv2.FONT_HERSHEY_DUPLEX,
                0.4,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

        if status_text is not None:
            cv2.putText(
                debug,
                status_text,
                (8, 18),
                cv2.FONT_HERSHEY_DUPLEX,
                0.4,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
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
