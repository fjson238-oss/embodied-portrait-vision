from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_id = LaunchConfiguration("device_id")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    fps = LaunchConfiguration("fps")
    camera_show_window = LaunchConfiguration("camera_show_window")
    detector_model_name = LaunchConfiguration("detector_model_name")
    detector_conf_threshold = LaunchConfiguration("detector_conf_threshold")
    detector_show_window = LaunchConfiguration("detector_show_window")
    image_topic = LaunchConfiguration("image_topic")
    detection_topic = LaunchConfiguration("detection_topic")
    skeleton_topic = LaunchConfiguration("skeleton_topic")
    composition_score_topic = LaunchConfiguration("composition_score_topic")
    skeleton_debug_image_topic = LaunchConfiguration("skeleton_debug_image_topic")
    skeleton_publish_debug_image = LaunchConfiguration("skeleton_publish_debug_image")
    skeleton_model_path = LaunchConfiguration("skeleton_model_path")
    skeleton_num_poses = LaunchConfiguration("skeleton_num_poses")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_id", default_value="0"),
            DeclareLaunchArgument("image_width", default_value="1280"),
            DeclareLaunchArgument("image_height", default_value="720"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("camera_show_window", default_value="false"),
            DeclareLaunchArgument("detector_model_name", default_value="yolov8n.onnx"),
            DeclareLaunchArgument("detector_conf_threshold", default_value="0.35"),
            DeclareLaunchArgument("detector_show_window", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("detection_topic", default_value="/person/detections"),
            DeclareLaunchArgument("skeleton_topic", default_value="/portrait/skeletons"),
            DeclareLaunchArgument("composition_score_topic", default_value="/composition/score"),
            DeclareLaunchArgument(
                "skeleton_debug_image_topic",
                default_value="/portrait/skeleton_debug_image",
            ),
            DeclareLaunchArgument("skeleton_publish_debug_image", default_value="true"),
            DeclareLaunchArgument("skeleton_model_path", default_value=""),
            DeclareLaunchArgument("skeleton_num_poses", default_value="1"),
            Node(
                package="vision_camera",
                executable="camera_node",
                name="camera_publisher",
                output="screen",
                parameters=[
                    {
                        "device_id": device_id,
                        "image_width": image_width,
                        "image_height": image_height,
                        "fps": fps,
                        "show_window": camera_show_window,
                    }
                ],
            ),
            Node(
                package="vision_detector",
                executable="detector_ort_node",
                name="detector_ort_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "model_name": detector_model_name,
                        "conf_threshold": detector_conf_threshold,
                        "detection_topic": detection_topic,
                        "show_window": detector_show_window,
                    }
                ],
            ),
            Node(
                package="composition_analyzer",
                executable="composition_analyzer_node",
                name="composition_analyzer_node",
                output="screen",
                parameters=[
                    {
                        "detection_topic": detection_topic,
                        "skeleton_topic": skeleton_topic,
                        "score_topic": composition_score_topic,
                    }
                ],
            ),
            Node(
                package="vision_skeleton",
                executable="skeleton_node",
                name="skeleton_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "skeleton_topic": skeleton_topic,
                        "debug_image_topic": skeleton_debug_image_topic,
                        "publish_debug_image": skeleton_publish_debug_image,
                        "model_path": skeleton_model_path,
                        "num_poses": skeleton_num_poses,
                    }
                ],
            ),
        ]
    )
