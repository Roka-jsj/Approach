# STELLA Approach Docker

This Docker image builds only the packages inside this `approach` folder:

- `yolo_msgs`
- `stella_approach`

The container runs the direct `cmd_vel` approach controller. It does not run object detection, camera drivers, Nav2, Gazebo, or any fallback navigation code.

## Build

```bash
cd ~/ros2_ws/src/approach
docker build -f Dockerfile.approach -t stella-approach:approach-only .
```

On Jetson Orin, use the same command. Docker will build the ARM64 image on the Jetson.

## Run

```bash
docker run --rm -it --network host \
  -e ROS_DOMAIN_ID=10 \
  stella-approach:approach-only
```

The container entrypoint sources:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
```

## Start

```bash
ros2 service call /approach_object stella_approach/srv/ApproachObject "{target_class: red_cube}"
```

The controller requires `/yolo/detections_3d` to contain detections whose `class_name` matches `target_class`. If `bbox3d.frame_id` is not `base_link`, the controller ignores that detection and waits for a `base_link` detection.
