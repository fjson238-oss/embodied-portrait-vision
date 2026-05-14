#!/usr/bin/env python3

import os

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from portrait_interfaces.msg import BoundingBox2D, Detection2D, DetectionArray
from rclpy.node import Node
from sensor_msgs.msg import Image

class DetectorOrtNode(Node):
    def __init__(self):
        super().__init__("detector_ort_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_name", "yolov8n.onnx")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("detection_topic", "/person/detections")
        self.declare_parameter("show_window", True)

        image_topic = self.get_parameter("image_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        self.show_window = bool(self.get_parameter("show_window").value)
        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_name = self.get_parameter("model_name").value
            model_path = os.path.join(
                get_package_share_directory("vision_detector"),
                "models",
                model_name,
            )

        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"YOLO model file does not exist: {model_path}")

        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.nms_threshold = float(self.get_parameter("nms_threshold").value)

        self.get_logger().info(f"Loading YOLO model with ONNX Runtime: {model_path}")
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        self.input_height = int(input_shape[2])
        self.input_width = int(input_shape[3])

        self.image_subscriber = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.detection_publisher = self.create_publisher(
            DetectionArray,
            detection_topic,
            10,
        )

        self.frame_count = 0
        self.get_logger().info("YOLO model loaded successfully.")
        self.get_logger().info(f"Subscribing image topic: {image_topic}")
        self.get_logger().info(f"Publishing person detections topic: {detection_topic}")
        self.get_logger().info(f"Detector preview window enabled: {self.show_window}")

    def image_callback(self, msg):
        try:
            frame = self.ros_image_to_bgr(msg)
            input_tensor = self.preprocess(frame)
            outputs = self.session.run(None, {self.input_name: input_tensor})
            detections = self.postprocess(outputs[0], frame.shape[1], frame.shape[0])

            result = frame.copy()
            for x1, y1, x2, y2, score in detections:
                cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    result,
                    f"person {score:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Received image: {frame.shape[1]} x {frame.shape[0]}, "
                    f"person detections: {len(detections)}"
                )
                
            detection_msg = self.create_detection_msg(msg, detections)
            self.detection_publisher.publish(detection_msg)

            if self.show_window:
                cv2.imshow("detector_ort_result", result)
                cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f"Detector error: {exc}")

    def create_detection_msg(self, image_msg, detections):
        msg = DetectionArray()
        msg.header = image_msg.header
        msg.frame_width = int(image_msg.width)
        msg.frame_height = int(image_msg.height)

        for x1, y1, x2, y2, score in detections:
            bbox = BoundingBox2D()
            bbox.x1 = int(x1)
            bbox.y1 = int(y1)
            bbox.x2 = int(x2)
            bbox.y2 = int(y2)
            bbox.width = int(x2 - x1)
            bbox.height = int(y2 - y1)
            bbox.center_x = float((x1 + x2) / 2.0)
            bbox.center_y = float((y1 + y2) / 2.0)

            detection = Detection2D()
            detection.class_name = "person"
            detection.class_id = 0
            detection.score = float(score)
            detection.bbox = bbox
            msg.detections.append(detection)

        return msg

    def ros_image_to_bgr(self, msg):
        height = msg.height
        width = msg.width
        encoding = msg.encoding.lower()

        if encoding in ("bgr8", "rgb8"):
            channels = 3
        elif encoding in ("bgra8", "rgba8"):
            channels = 4
        elif encoding in ("mono8", "8uc1"):
            channels = 1
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape((height, msg.step))
        image = image[:, : width * channels]
        image = image.reshape((height, width, channels))

        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding in ("mono8", "8uc1"):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image.copy()

    def preprocess(self, frame):
        resized = cv2.resize(frame, (self.input_width, self.input_height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))
        return np.expand_dims(tensor, axis=0)

    def postprocess(self, output, frame_width, frame_height):
        predictions = np.squeeze(output)
        if predictions.ndim != 2:
            return []
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        boxes = []
        scores = []
        scale_x = frame_width / self.input_width
        scale_y = frame_height / self.input_height

        for prediction in predictions:
            class_scores = prediction[4:]
            person_score = float(class_scores[0])
            if person_score < self.conf_threshold:
                continue

            cx, cy, width, height = prediction[:4]
            x1 = int((cx - width / 2.0) * scale_x)
            y1 = int((cy - height / 2.0) * scale_y)
            w = int(width * scale_x)
            h = int(height * scale_y)

            boxes.append([x1, y1, w, h])
            scores.append(person_score)

        indices = cv2.dnn.NMSBoxes(
            boxes,
            scores,
            self.conf_threshold,
            self.nms_threshold,
        )

        detections = []
        for index in np.array(indices).flatten():
            x, y, w, h = boxes[index]
            x1 = max(0, x)
            y1 = max(0, y)
            x2 = min(frame_width - 1, x + w)
            y2 = min(frame_height - 1, y + h)
            detections.append((x1, y1, x2, y2, scores[index]))
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = DetectorOrtNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
