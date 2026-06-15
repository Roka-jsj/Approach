# STELLA Approach

This module runs only the direct `cmd_vel` object approach controller.

It does not use Nav2, map coordinates, `/yolo/get_target_position`, or fallback navigation logic.

## Runtime Contract

- Approach service: `/approach_object` (`stella_approach/srv/ApproachObject`)
- Input topic: `/yolo/detections_3d` (`yolo_msgs/msg/DetectionArray`)
- Output topic: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Expected approach frame: `base_link`
- Detection frames other than `base_link` are ignored.
- Required service field: `target_class`

If service `target_class` is empty, the service rejects the request and the robot does not move.

## Behavior

1. Wait for a detection matching `target_class`.
2. Use only detections that already arrive in the configured target frame, `base_link` by default.
3. If the object distance is already `0.30m` or less, stop and finish.
4. Rotate toward the object using at most `10deg` per rotation step.
5. Stop for `0.2s` after each rotation and wait for a fresh detection.
6. When the object is centered within `y_tolerance`, drive forward with small angular correction from the latest `y` error.
7. If the target disappears while driving, stop and wait for a fresh detection.
8. If the target drifts too far while driving, stop forward motion and realign.
9. Stop when the object distance becomes `0.30m` or less.

The controller never publishes reverse linear velocity.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src/approach/yolo_msgs src/approach/stella_approach \
  --packages-select yolo_msgs stella_approach
source install/setup.bash
```

## Run

```bash
ros2 launch stella_approach approach_goal.launch.py
```

The default launch values are tuned for the real robot:

```bash
ros2 launch stella_approach approach_goal.launch.py
```

Start approach:

```bash
ros2 service call /approach_object stella_approach/srv/ApproachObject "{target_class: red_cube}"
```

## Docker Build

```bash
cd ~/ros2_ws/src/approach
docker build -f Dockerfile.approach -t stella-approach:approach-only .
```

## Docker Run

```bash
docker run --rm -it --network host \
  -e ROS_DOMAIN_ID=10 \
  stella-approach:approach-only
```

Use the same `ROS_DOMAIN_ID` as the camera, object detection, and Stella base containers.
