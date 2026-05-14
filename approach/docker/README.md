# STELLA Approach Container

이 이미지는 `stella_approach` 노드만 빌드하고 실행합니다. Nav2와 object_detection은 같은 `ROS_DOMAIN_ID`, 같은 `RMW_IMPLEMENTATION`, `--network host` 조건에서 별도 컨테이너나 노드로 실행되어 있어야 합니다.

## Build

Jetson에서 `nvidia@nvidia-desktop:~/approach$` 위치에서 실행하는 기준입니다. 이 디렉터리가 Docker build context이며, 컨테이너 안에서는 `/home/nvidia/ros2_ws/src/approach`로 복사되어 `ros2_ws/src` 아래에 `approach`가 있는 구조로 빌드됩니다.

예상 host 디렉터리 구조는 아래처럼 두면 됩니다.

```text
~/approach
├── Dockerfile.approach
├── docker/
├── stella_approach/
└── yolo_msgs/
```

```bash
cd ~/approach
docker build --network host --no-cache -f Dockerfile.approach -t stella-approach:humble-orin .
```

Dockerfile의 실제 build source 경로는 아래와 같습니다.

```bash
COPY . /home/nvidia/ros2_ws/src/approach
cd /home/nvidia/ros2_ws
colcon build \
  --base-paths /home/nvidia/ros2_ws/src/approach \
  --packages-up-to stella_approach \
  --build-base /home/nvidia/ros2_ws/build \
  --install-base /home/nvidia/ros2_ws/install \
  --symlink-install
```

`--no-cache`는 이전에 깨진 image layer가 남아 있을 때 확실히 새로 빌드하기 위한 옵션입니다. 한 번 정상 빌드된 뒤에는 필요하면 생략해도 됩니다.

이 build context에는 `stella_approach`와 `yolo_msgs` 인터페이스 패키지가 함께 들어 있습니다. `yolo_msgs`는 apt 패키지가 아닐 수 있으므로, 이 복사본을 제거한다면 같은 메시지 정의가 들어 있는 underlay를 먼저 빌드하고 source해야 합니다.

## Run

```bash
docker run --rm -it --network host --ipc host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  stella-approach:humble-orin
```

컨테이너 내부 workspace root는 `/home/nvidia/ros2_ws`입니다. source는 `/home/nvidia/ros2_ws/src/approach`, install space는 `/home/nvidia/ros2_ws/install`입니다. 수동 셸에서 ROS 환경을 다시 잡아야 하면 아래처럼 source합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
```

## Parameter Trigger

노드가 실행 중인 상태에서 다른 터미널 또는 같은 ROS 네트워크의 컨테이너에서 실행합니다.

```bash
ros2 param set /approach_goal_service target_class_name person
ros2 param set /approach_goal_service start_approach true
```

`start_approach`가 `true`로 감지되면 한 번만 실행하고 노드가 다시 `false`로 되돌립니다.

## Service Trigger

```bash
ros2 service call /approach_target stella_approach/srv/ApproachTarget "{class_name: person}"
```

service 이름은 launch/parameter의 `service_name` 값으로 바꿀 수 있습니다.

## Auto Start

```bash
ros2 launch stella_approach approach_goal.launch.py \
  target_class_name:=person \
  auto_start_on_launch:=true
```

`auto_start_on_launch`는 노드 시작 후 한 번만 확인합니다.

## Runtime Requirements

- object_detection service: `/yolo/get_target_position` (`yolo_msgs/srv/GetTargetPosition`)
- detection topic: `/yolo/detections_3d` (`yolo_msgs/msg/DetectionArray`)
- Nav2 action server: `/navigate_to_pose` 또는 `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- TF: `map -> base_link`

object_detection, Nav2, TF가 준비되어 있지 않으면 approach 요청은 timeout 또는 lookup 실패로 종료됩니다.
