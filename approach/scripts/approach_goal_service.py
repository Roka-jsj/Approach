#!/usr/bin/env python3

import math
import threading
from functools import partial

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from stella_approach.srv import ApproachTarget
from yolo_msgs.msg import DetectionArray
from yolo_msgs.srv import GetTargetPosition


class ApproachGoalService(Node):

    def __init__(self):
        super().__init__('approach_goal_service')

        self.declare_parameter('service_name', 'approach_target')
        self.declare_parameter('navigate_action_name', 'navigate_to_pose')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('detections_topic', '/yolo/detections_3d')
        self.declare_parameter(
            'object_position_service_name',
            '/yolo/get_target_position')
        self.declare_parameter('action_server_timeout_sec', 2.0)
        self.declare_parameter('object_service_timeout_sec', 2.0)
        self.declare_parameter('transform_timeout_sec', 2.0)
        self.declare_parameter('target_class_name', '')
        self.declare_parameter('start_approach', False)
        self.declare_parameter('auto_start_on_launch', False)
        self.declare_parameter('parameter_poll_period_sec', 0.2)

        self.service_name = self.get_parameter('service_name').value
        self.navigate_action_name = self.get_parameter('navigate_action_name').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.object_position_service_name = self.get_parameter(
            'object_position_service_name').value
        self.action_server_timeout_sec = float(
            self.get_parameter('action_server_timeout_sec').value)
        self.object_service_timeout_sec = float(
            self.get_parameter('object_service_timeout_sec').value)
        self.transform_timeout_sec = float(
            self.get_parameter('transform_timeout_sec').value)
        parameter_poll_period_sec = float(
            self.get_parameter('parameter_poll_period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_action_name)
        self.object_position_client = self.create_client(
            GetTargetPosition,
            self.object_position_service_name)
        self.detections_sub = self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.handle_detections_3d,
            10)

        self.pending_futures = set()
        self.latest_detections = []
        self.latest_detection_stamp = None
        self.latest_target_map_position = None
        self.active_target_name = None
        self.approach_in_progress = False
        self.auto_start_checked = False
        self.state_lock = threading.Lock()
        self.service = self.create_service(
            ApproachTarget,
            self.service_name,
            self.handle_approach_target)
        self.parameter_timer = self.create_timer(
            parameter_poll_period_sec,
            self.handle_parameter_trigger)

        self.get_logger().info(
            f'Ready for {self.service_name} requests: '
            f'detections_topic={self.detections_topic}, '
            f'object_position_service={self.object_position_service_name}, '
            f'action={self.navigate_action_name}')

    def handle_approach_target(self, request, response):
        accepted, message = self.start_approach(request.class_name)
        response.accepted = accepted
        response.message = message
        return response

    def handle_parameter_trigger(self):
        auto_start_on_launch = bool(
            self.get_parameter('auto_start_on_launch').value)
        start_approach = bool(self.get_parameter('start_approach').value)

        if auto_start_on_launch and not self.auto_start_checked:
            self.auto_start_checked = True
            start_approach = True

        if not start_approach:
            return

        if bool(self.get_parameter('start_approach').value):
            self.set_parameters([
                Parameter('start_approach', Parameter.Type.BOOL, False)])

        class_name = self.get_parameter('target_class_name').value
        accepted, message = self.start_approach(class_name)
        if accepted:
            self.get_logger().info(f'Parameter trigger accepted: {message}')
        else:
            self.get_logger().error(f'Parameter trigger rejected: {message}')

    def start_approach(self, class_name):
        class_name = str(class_name).strip()
        if not class_name:
            return False, 'target_class_name/class_name is empty'

        with self.state_lock:
            if self.approach_in_progress:
                return False, (
                    f'Approach already in progress for {self.active_target_name}')

        if not self.nav_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            return False, (
                f'Nav2 action server {self.navigate_action_name} is not available')

        if not self.object_position_client.wait_for_service(
            timeout_sec=self.object_service_timeout_sec
        ):
            return False, (
                f'Object position service {self.object_position_service_name} '
                'is not available')

        with self.state_lock:
            self.approach_in_progress = True
            self.active_target_name = class_name

        service_request = GetTargetPosition.Request()
        service_request.class_name = class_name
        future = self.object_position_client.call_async(service_request)
        self.track_future(
            future,
            partial(self.handle_object_position_response, class_name=class_name))

        return True, f'Requested {self.object_position_service_name} for {class_name}'

    def handle_detections_3d(self, msg):
        detections = []
        for detection in msg.detections:
            bbox3d = detection.bbox3d
            detections.append({
                'class_name': detection.class_name,
                'score': float(detection.score),
                'frame_id': bbox3d.frame_id,
                'x': float(bbox3d.center.position.x),
                'y': float(bbox3d.center.position.y),
                'z': float(bbox3d.center.position.z),
                'distance': float(bbox3d.distance),
            })

        self.latest_detections = detections
        self.latest_detection_stamp = self.get_clock().now()

        self.get_logger().debug(
            f'Received {len(detections)} detections from {self.detections_topic}')

    def handle_object_position_response(self, future, class_name):
        self.pending_futures.discard(future)
        try:
            result = future.result()
            if result is None:
                raise RuntimeError(
                    f'No response from {self.object_position_service_name}')
            if not result.success:
                raise RuntimeError(
                    f'Target {class_name} was not found by '
                    f'{self.object_position_service_name}')

            frame_id = result.frame_id
            if frame_id != self.target_frame:
                raise RuntimeError(
                    f'Expected {self.target_frame} target, got {frame_id}')

            target = {
                'class_name': class_name,
                'frame_id': frame_id,
                'x': float(result.x),
                'y': float(result.y),
                'z': float(result.z),
                'distance': float(result.distance),
                'stamp': self.get_clock().now(),
            }
            self.latest_target_map_position = target

            robot_x, robot_y = self.lookup_robot_xy()
            goal_x, goal_y, goal_yaw = self.compute_goal_pose(
                target['x'],
                target['y'],
                robot_x,
                robot_y)
            self.send_navigation_goal(
                goal_x,
                goal_y,
                goal_yaw,
                class_name)
        except TransformException as exc:
            self.fail_approach(f'TF lookup failed: {exc}')
        except Exception as exc:
            self.fail_approach(
                f'Failed to approach {class_name}: {exc}')

    def lookup_robot_xy(self):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=self.transform_timeout_sec))
        translation = transform.transform.translation
        return translation.x, translation.y

    def compute_goal_pose(self, target_x, target_y, robot_x, robot_y):
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)

        if distance < 1e-6:
            return target_x, target_y, 0.0

        yaw = math.atan2(dy, dx)
        return target_x, target_y, yaw

    def send_navigation_goal(self, goal_x, goal_y, goal_yaw, class_name):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self.quaternion_from_yaw(goal_yaw)

        future = self.nav_client.send_goal_async(goal)
        self.track_future(future, self.handle_goal_response)

        self.get_logger().info(
            f'Sent {class_name} goal ({goal_x:.3f}, {goal_y:.3f}, '
            f'yaw={goal_yaw:.3f}) in {self.target_frame}')

    @staticmethod
    def quaternion_from_yaw(yaw):
        half_yaw = yaw * 0.5
        quaternion = Quaternion()
        quaternion.z = math.sin(half_yaw)
        quaternion.w = math.cos(half_yaw)
        return quaternion

    def handle_goal_response(self, future):
        self.pending_futures.discard(future)
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.fail_approach(f'Failed to send Nav2 goal: {exc}')
            return

        if not goal_handle.accepted:
            self.fail_approach('Nav2 rejected the approach goal')
            return

        self.get_logger().info('Nav2 accepted the approach goal')
        result_future = goal_handle.get_result_async()
        self.track_future(result_future, self.handle_navigation_result)

    def handle_navigation_result(self, future):
        self.pending_futures.discard(future)
        try:
            result = future.result()
            status_name = self.goal_status_name(result.status)
            self.get_logger().info(
                f'Navigation finished with status: {status_name}')
        except Exception as exc:
            self.get_logger().error(f'Navigation result failed: {exc}')
        finally:
            self.clear_approach_state()

    def fail_approach(self, message):
        self.get_logger().error(message)
        self.clear_approach_state()

    def clear_approach_state(self):
        with self.state_lock:
            self.approach_in_progress = False
            self.active_target_name = None

    def track_future(self, future, callback):
        self.pending_futures.add(future)
        future.add_done_callback(callback)

    @staticmethod
    def goal_status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, str(status))


def main(args=None):
    rclpy.init(args=args)
    node = ApproachGoalService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
