import cv2
import mediapipe as mp
import os

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python import vision
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from portrait_interfaces.msg import (
    SkeletonArray,
    Skeleton2D,
    SkeletonKeypoint2D,
)


class SkeletonNode(Node):
    def __init__(self):
        super().__init__("skeleton_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("skeleton_topic", "/portrait/skeletons")
        self.declare_parameter("debug_image_topic", "/portrait/skeleton_debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("min_visibility", 0.3)
        self.declare_parameter("model_path", "")
        self.declare_parameter("num_poses", 1)
        self.declare_parameter("min_pose_detection_confidence", 0.5)
        self.declare_parameter("min_pose_presence_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        self.image_topic = self.get_parameter("image_topic").value
        self.skeleton_topic = self.get_parameter("skeleton_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.min_visibility = float(self.get_parameter("min_visibility").value)
        self.model_path = self.get_parameter("model_path").value
        self.num_poses = int(self.get_parameter("num_poses").value)
        self.min_pose_detection_confidence = float(
            self.get_parameter("min_pose_detection_confidence").value
        )
        self.min_pose_presence_confidence = float(
            self.get_parameter("min_pose_presence_confidence").value
        )
        self.min_tracking_confidence = float(
            self.get_parameter("min_tracking_confidence").value
        )

        if not self.model_path:
            self.model_path = os.path.join(
                get_package_share_directory("vision_skeleton"),
                "models",
                "pose_landmarker_lite.task",
            )

        if not self.model_path or not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                "MediaPipe Pose Landmarker requires a .task model file. "
                "Run with: ros2 run vision_skeleton skeleton_node --ros-args "
                "-p model_path:=/absolute/path/to/pose_landmarker.task"
            )

        self.bridge = CvBridge()

        self.landmark_names = [
            landmark.name.lower() for landmark in vision.PoseLandmark
        ]
        self.pose_connections = vision.PoseLandmarksConnections.POSE_LANDMARKS
        options = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=self.num_poses,
            min_pose_detection_confidence=self.min_pose_detection_confidence,
            min_pose_presence_confidence=self.min_pose_presence_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
            output_segmentation_masks=False,
        )
        self.pose_landmarker = vision.PoseLandmarker.create_from_options(options)

        self.skeleton_pub = self.create_publisher(
            SkeletonArray,
            self.skeleton_topic,
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.frame_count = 0

        self.get_logger().info(f"Skeleton node started.")
        self.get_logger().info(f"Subscribe image topic: {self.image_topic}")
        self.get_logger().info(f"Publish skeleton topic: {self.skeleton_topic}")
        self.get_logger().info(f"Using MediaPipe Tasks model: {self.model_path}")

    def image_callback(self, msg: Image):
        self.frame_count += 1

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS image to OpenCV image: {e}")
            return

        height, width = frame_bgr.shape[:2]

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        timestamp_ms = self.ros_time_to_ms(msg.header.stamp)
        result = self.pose_landmarker.detect_for_video(mp_image, timestamp_ms)

        skeleton_array_msg = SkeletonArray()
        skeleton_array_msg.header = msg.header
        skeleton_array_msg.frame_width = int(width)
        skeleton_array_msg.frame_height = int(height)

        for person_id, pose_landmarks in enumerate(result.pose_landmarks):
            skeleton_msg = self.build_skeleton_msg(
                pose_landmarks,
                msg.header,
                width,
                height,
                person_id,
            )

            skeleton_array_msg.skeletons.append(skeleton_msg)

        if self.publish_debug_image and result.pose_landmarks:
            self.draw_debug_image(frame_bgr, skeleton_array_msg.skeletons, msg.header)

        self.skeleton_pub.publish(skeleton_array_msg)

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Processed {self.frame_count} frames, "
                f"detected skeletons: {len(skeleton_array_msg.skeletons)}"
            )

    def ros_time_to_ms(self, stamp):
        return int(stamp.sec * 1000 + stamp.nanosec / 1_000_000)

    def build_skeleton_msg(self, pose_landmarks, header, width, height, person_id):
        skeleton_msg = Skeleton2D()
        skeleton_msg.header = header
        skeleton_msg.person_id = int(person_id)
        skeleton_msg.has_pose = True

        valid_xs = []
        valid_ys = []
        valid_scores = []

        for idx, landmark in enumerate(pose_landmarks):
            keypoint_msg = SkeletonKeypoint2D()

            keypoint_msg.id = idx

            if idx < len(self.landmark_names):
                keypoint_msg.name = self.landmark_names[idx]
            else:
                keypoint_msg.name = f"point_{idx}"

            keypoint_msg.x = float(landmark.x * width)
            keypoint_msg.y = float(landmark.y * height)
            keypoint_msg.score = float(getattr(landmark, "visibility", 0.0))
            keypoint_msg.valid = keypoint_msg.score >= self.min_visibility

            if keypoint_msg.valid:
                valid_xs.append(keypoint_msg.x)
                valid_ys.append(keypoint_msg.y)
                valid_scores.append(keypoint_msg.score)

            skeleton_msg.keypoints.append(keypoint_msg)

        if valid_xs and valid_ys:
            x_min = max(0.0, min(valid_xs))
            y_min = max(0.0, min(valid_ys))
            x_max = min(float(width - 1), max(valid_xs))
            y_max = min(float(height - 1), max(valid_ys))

            skeleton_msg.bbox_x = float(x_min)
            skeleton_msg.bbox_y = float(y_min)
            skeleton_msg.bbox_width = float(x_max - x_min)
            skeleton_msg.bbox_height = float(y_max - y_min)
            skeleton_msg.bbox.x1 = int(x_min)
            skeleton_msg.bbox.y1 = int(y_min)
            skeleton_msg.bbox.x2 = int(x_max)
            skeleton_msg.bbox.y2 = int(y_max)
            skeleton_msg.bbox.width = int(x_max - x_min)
            skeleton_msg.bbox.height = int(y_max - y_min)
            skeleton_msg.bbox.center_x = float((x_min + x_max) / 2.0)
            skeleton_msg.bbox.center_y = float((y_min + y_max) / 2.0)

            skeleton_msg.score = float(sum(valid_scores) / len(valid_scores))
        else:
            skeleton_msg.has_pose = False
            skeleton_msg.score = 0.0
            skeleton_msg.bbox_x = 0.0
            skeleton_msg.bbox_y = 0.0
            skeleton_msg.bbox_width = 0.0
            skeleton_msg.bbox_height = 0.0

        return skeleton_msg

    def draw_debug_image(self, frame_bgr, skeletons, header):
        for skeleton in skeletons:
            for connection in self.pose_connections:
                start = skeleton.keypoints[connection.start]
                end = skeleton.keypoints[connection.end]
                if not start.valid or not end.valid:
                    continue
                cv2.line(
                    frame_bgr,
                    (int(start.x), int(start.y)),
                    (int(end.x), int(end.y)),
                    (0, 255, 255),
                    2,
                )

            for keypoint in skeleton.keypoints:
                if not keypoint.valid:
                    continue
                cv2.circle(
                    frame_bgr,
                    (int(keypoint.x), int(keypoint.y)),
                    3,
                    (0, 255, 0),
                    -1,
                )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

    def destroy_node(self):
        self.pose_landmarker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SkeletonNode()

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
