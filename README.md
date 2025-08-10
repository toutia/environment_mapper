# environment\_mapper

**Environment Mapper** is a ROS2-based toolkit that integrates camera/IMU input, NVIDIA DeepStream object detection, and RTAB-Map semantic mapping. It provides launch files for sensors and mapping tools, plus an `object_detection` package that uses DeepStream to detect and track objects, then labels them in RTAB-Map maps.

## Repository Contents

- \`\` — ROS2 launch scripts and detailed build notes for CUDA, DeepStream, RTAB-Map, and RealSense.
- \`\` — A ROS2 ament package containing:
  - DeepStream GStreamer pipeline (`detection_pipeline.py`)
  - ROS2 nodes: `object_detector`, `object_labeler`, and `rgbd_merger`
  - Pre-trained models/configs under `Primary_Detector/`
- \`\` — Utilities like a Kalibr Dockerfile.

## Data Flow

1. Camera node publishes RGB and depth.
2. `object_detector` subscribes, converts frames via `cv_bridge`, and pushes them to DeepStream (`appsrc`).
3. DeepStream runs detection (`nvinfer`), tracking (`nvtracker`), and annotation (`nvdsosd`), then Python probes extract metadata with `pyds`.
4. Depth is averaged per detection ROI (if available).
5. Detections are published to `/detections`, consumed by `object_labeler`, which adds labels to RTAB-Map and publishes `PoseStamped` goals.

## Quickstart

1. Install ROS2 Humble, NVIDIA drivers + CUDA, DeepStream SDK + Python bindings, librealsense2, and RTAB-Map.
2. Clone into a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/toutia/environment_mapper.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

3. Run your chosen launch files for sensors and RTAB-Map, then start:

```bash
ros2 run object_detection object_detector
ros2 run object_detection object_labeler
```

## Requirements & Caveats

- Requires NVIDIA GPU + DeepStream SDK.
- Model engine files may be platform-specific; rebuild if needed.
- ROS2-focused; some legacy ROS1 references exist.

## Suggested Improvements

- Add `requirements.txt` and Dockerfile for reproducible builds.
- Provide lightweight example bag files for testing without hardware.
- Document exact topic names/parameters for all nodes.

## License

- `object_detection` is under Apache-2.0; DeepStream configs/models follow NVIDIA licensing.

