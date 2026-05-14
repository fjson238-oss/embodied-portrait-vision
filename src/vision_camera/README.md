# vision_camera

Minimal ROS 2 camera viewer.

## Run

```bash
ros2 run vision_camera camera_node --ros-args -p device_id:=0 -p window_name:=camera
```

This version uses `V4L2 + MJPG + 1280x720 + grab/retrieve` because that is the working combo for the current camera.
