# Architecture

本文档记录当前 MVP 的 ROS2 节点架构、数据流和后续扩展方向。

## MVP Data Flow

```mermaid
flowchart TD
    subgraph Input
        CameraNode[vision_camera/camera_node]
    end

    subgraph Perception
        Detector[vision_detector/detector_ort_node<br/>YOLO ONNX Runtime]
        Skeleton[vision_skeleton/skeleton_node<br/>MediaPipe Pose Landmarker]
    end

    subgraph Analysis
        Analyzer[composition_analyzer/composition_analyzer_node]
        Rules[CompositionRules<br/>ID-photo composition rules]
    end

    subgraph Output
        Score[/composition/score<br/>CompositionScore/]
        Suggestion[/composition/suggestion<br/>String/]
        DebugImage[/portrait/skeleton_debug_image<br/>Image/]
    end

    CameraNode -->|/camera/image_raw| Detector
    CameraNode -->|/camera/image_raw| Skeleton
    Detector -->|/person/detections| Analyzer
    Skeleton -->|/portrait/skeletons| Analyzer
    Skeleton --> DebugImage
    Analyzer --> Rules
    Rules --> Analyzer
    Analyzer --> Score
    Analyzer --> Suggestion
```

## Node Responsibilities

| Package | Node | Responsibility |
| --- | --- | --- |
| `vision_camera` | `camera_node` | Capture camera frames and publish `sensor_msgs/Image` |
| `vision_detector` | `detector_ort_node` | Run YOLO ONNX Runtime inference and publish person detections |
| `vision_skeleton` | `skeleton_node` | Run MediaPipe Pose Landmarker and publish skeleton keypoints |
| `composition_analyzer` | `composition_analyzer_node` | Fuse detections and skeletons, call composition rules, publish score and advice |
| `portrait_interfaces` | N/A | Define shared ROS2 msg types |
| `embodied_vision_bringup` | launch | Start the full MVP pipeline |

## Message Interfaces

```mermaid
classDiagram
    class DetectionArray {
      Header header
      int32 frame_width
      int32 frame_height
      Detection2D[] detections
    }

    class Detection2D {
      string class_name
      int32 class_id
      float32 score
      BoundingBox2D bbox
    }

    class SkeletonArray {
      Header header
      int32 frame_width
      int32 frame_height
      Skeleton2D[] skeletons
    }

    class Skeleton2D {
      Header header
      uint32 person_id
      bool has_pose
      float32 score
      BoundingBox2D bbox
      SkeletonKeypoint2D[] keypoints
    }

    class CompositionScore {
      Header header
      int32 score
      string level
      string profile
      string summary
      string[] suggestions
      int32 frame_width
      int32 frame_height
      uint32 detection_count
      uint32 skeleton_count
    }

    DetectionArray --> Detection2D
    Detection2D --> BoundingBox2D
    SkeletonArray --> Skeleton2D
    Skeleton2D --> BoundingBox2D
    Skeleton2D --> SkeletonKeypoint2D
```

## Runtime Topics

```text
/camera/image_raw
  sensor_msgs/msg/Image

/person/detections
  portrait_interfaces/msg/DetectionArray

/portrait/skeletons
  portrait_interfaces/msg/SkeletonArray

/portrait/skeleton_debug_image
  sensor_msgs/msg/Image

/composition/score
  portrait_interfaces/msg/CompositionScore

/composition/suggestion
  std_msgs/msg/String
```

## Launch

Main launch file:

```text
src/embodied_vision_bringup/launch/composition_pipeline.launch.py
```

Run:

```bash
source install/setup.bash
ros2 launch embodied_vision_bringup composition_pipeline.launch.py
```

## Rule Layer

The node layer converts ROS2 messages into plain C++ structures:

```text
DetectionArray -> std::vector<PersonBox>
SkeletonArray  -> std::vector<SkeletonInfo>
```

Then it calls:

```cpp
rules_.analyze_id_photo(frame, person_boxes, latest_skeletons_);
```

Rule implementation lives in:

```text
src/composition_analyzer/src/composition_rules.cpp
```

Current rules:

- Subject horizontal position
- Subject scale in frame
- Shoulder line levelness
- Body axis tilt
- Head-to-shoulder alignment
- Headroom placeholder for future face/head estimation

## Future ROS2-Robotics Extensions

The next stage should make ROS2 more central by adding spatial perception:

```mermaid
flowchart LR
    CameraInfo[/camera/camera_info/] --> Spatial[spatial_perception_node]
    Detection[/person/detections/] --> Spatial
    TF[/tf/] --> Spatial
    Spatial -->|/person/spatial_state| Composition[/composition/score/]
    Spatial -->|yaw/pitch error| Gimbal[gimbal_controller_node]
    Composition --> Agent[LLM advisor_node]
```

Recommended additions:

- Publish `sensor_msgs/CameraInfo` from the camera node.
- Convert image-space person center into normalized camera ray.
- Compute yaw/pitch error from camera intrinsics.
- Add TF frames such as `base_link`, `gimbal_link`, `camera_link`, `camera_optical_frame`.
- Add a simulated or real `gimbal_controller_node`.
- Let an LLM Agent consume `/composition/score` and `/person/spatial_state` for personalized advice.
