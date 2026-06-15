#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from stella_approach.srv import ApproachObject
from yolo_msgs.msg import DetectionArray


@dataclass
class TargetSample:
    x: float
    y: float
    z: float
    distance: float
    score: float
    class_name: str
    frame_id: str
    stamp_sec: float


class ApproachController(Node):
    def __init__(self):
        super().__init__("approach_controller")

        self.declare_parameter("detections_topic", "/yolo/detections_3d")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("service_name", "approach_object")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("invert_y", False)
        self.declare_parameter("angular_direction_sign", 1.0)

        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("desired_distance", 0.30)
        self.declare_parameter("y_tolerance", 0.10)
        self.declare_parameter("use_3d_distance", False)
        self.declare_parameter("target_timeout_sec", 1.5)
        self.declare_parameter("drive_target_timeout_sec", 0.35)
        self.declare_parameter("wait_log_interval_sec", 5.0)

        self.declare_parameter("max_rotation_step_deg", 10.0)
        self.declare_parameter("rotate_speed", 0.8)
        self.declare_parameter("settle_time_sec", 0.2)

        self.declare_parameter("linear_kp", 0.45)
        self.declare_parameter("max_forward_speed", 0.2)
        self.declare_parameter("min_linear_speed", 0.02)
        self.declare_parameter("drive_angular_kp", 1.2)
        self.declare_parameter("max_drive_angular_speed", 0.35)
        self.declare_parameter("max_drive_heading_error_deg", 12.0)

        self._active = False
        self._state = "IDLE"
        self._state_start_sec = self._now_sec()
        self._rotate_end_sec = self._state_start_sec
        self._rotate_cmd = 0.0
        self._last_target: Optional[TargetSample] = None
        self._target_class = ""
        self._active_start_sec = self._state_start_sec
        self._last_warn_sec = 0.0
        self._last_wait_log_sec = 0.0

        detections_topic = self.get_parameter("detections_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        service_name = self.get_parameter("service_name").value
        control_frequency = float(self.get_parameter("control_frequency").value)

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._det_sub = self.create_subscription(
            DetectionArray, detections_topic, self._on_detections, 10
        )
        self._srv = self.create_service(ApproachObject, service_name, self._on_service)
        self._timer = self.create_timer(1.0 / max(control_frequency, 1.0), self._on_timer)

        self.get_logger().info(
            "Approach controller ready: service=%s, detections=%s, cmd_vel=%s"
            % (service_name, detections_topic, cmd_vel_topic)
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _norm_name(name: str) -> str:
        return "".join(c for c in name.lower() if c.isalnum())

    def _matches_target_class(self, class_name: str) -> bool:
        target_class = self._target_class.strip()
        if not target_class:
            return False
        query = self._norm_name(target_class)
        candidate = self._norm_name(class_name)
        return bool(query and candidate and candidate == query)

    def _on_detections(self, msg: DetectionArray) -> None:
        target_frame = str(self.get_parameter("target_frame").value)
        invert_y = bool(self.get_parameter("invert_y").value)
        use_3d_distance = bool(self.get_parameter("use_3d_distance").value)
        now = self._now_sec()

        best: Optional[TargetSample] = None
        for det in msg.detections:
            if not det.class_name or not self._matches_target_class(det.class_name):
                continue

            bbox3d = det.bbox3d
            frame_id = str(bbox3d.frame_id)
            x = float(bbox3d.center.position.x)
            y = float(bbox3d.center.position.y)
            z = float(bbox3d.center.position.z)
            if target_frame and frame_id != target_frame:
                self._warn_throttled(
                    "Ignoring detection '%s' in frame '%s'; expected '%s'"
                    % (det.class_name, frame_id, target_frame)
                )
                continue

            if invert_y:
                y = -y
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue

            distance = math.sqrt(x * x + y * y + z * z) if use_3d_distance else math.sqrt(x * x + y * y)
            sample = TargetSample(
                x=x,
                y=y,
                z=z,
                distance=distance,
                score=float(det.score),
                class_name=str(det.class_name),
                frame_id=frame_id,
                stamp_sec=now,
            )
            if best is None or sample.distance < best.distance:
                best = sample

        if best is not None:
            self._last_target = best
        elif self._active:
            self._last_target = None

    def _warn_throttled(self, message: str) -> None:
        now = self._now_sec()
        if now - self._last_warn_sec > 2.0:
            self._last_warn_sec = now
            self.get_logger().warn(message)

    def _on_service(self, request: ApproachObject.Request, response: ApproachObject.Response):
        target_class = str(request.target_class).strip()
        if not target_class:
            self._stop_robot()
            response.success = False
            response.message = "target_class is required"
            return response

        if self._active:
            response.success = False
            response.message = "approach already active"
            return response

        self._active = True
        self._target_class = target_class
        self._active_start_sec = self._now_sec()
        self._last_target = None
        self._set_state("ACQUIRE")
        self.get_logger().info("Approach started for %s" % target_class)

        response.success = True
        response.message = "approach started for %s" % target_class
        return response

    def _on_timer(self) -> None:
        if not self._active:
            return

        if self._state == "ACQUIRE":
            self._handle_acquire()
        elif self._state == "ROTATING":
            self._handle_rotating()
        elif self._state == "SETTLING":
            self._handle_settling()
        elif self._state == "DRIVING":
            self._handle_driving()

    def _set_state(self, state: str) -> None:
        self._state = state
        self._state_start_sec = self._now_sec()

    def _fresh_target(
        self,
        after_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> Optional[TargetSample]:
        if self._last_target is None:
            return None

        now = self._now_sec()
        timeout = (
            float(timeout_sec)
            if timeout_sec is not None
            else float(self.get_parameter("target_timeout_sec").value)
        )
        if now - self._last_target.stamp_sec > timeout:
            return None
        if after_sec is not None and self._last_target.stamp_sec < after_sec:
            return None
        return self._last_target

    def _handle_acquire(self) -> None:
        target = self._fresh_target()
        if target is None:
            self._stop_robot()
            self._log_waiting_for_target()
            return
        self._align_or_drive(target)

    def _log_waiting_for_target(self) -> None:
        now = self._now_sec()
        interval = float(self.get_parameter("wait_log_interval_sec").value)
        if now - self._last_wait_log_sec < max(interval, 1.0):
            return
        self._last_wait_log_sec = now
        self.get_logger().info(
            "Waiting for %s detection in %s"
            % (self._target_class, self.get_parameter("target_frame").value)
        )

    def _align_or_drive(self, target: TargetSample) -> None:
        desired = float(self.get_parameter("desired_distance").value)
        if target.distance <= desired:
            self._finish(
                True,
                "already within %.3fm of %s (distance=%.3f x=%.3f y=%.3f)"
                % (desired, target.class_name, target.distance, target.x, target.y),
            )
            return

        y_tolerance = float(self.get_parameter("y_tolerance").value)
        if abs(target.y) <= y_tolerance:
            self._set_state("DRIVING")
            self.get_logger().info(
                "Aligned with %s: x=%.3f y=%.3f distance=%.3f"
                % (target.class_name, target.x, target.y, target.distance)
            )
            return

        self._begin_rotation(target)

    def _begin_rotation(self, target: TargetSample) -> None:
        heading_error = math.atan2(target.y, target.x)
        max_step = math.radians(float(self.get_parameter("max_rotation_step_deg").value))
        step_angle = self._clamp(heading_error, -max_step, max_step)

        if abs(step_angle) < 1e-4:
            self._set_state("SETTLING")
            self._stop_robot()
            return

        rotate_speed = abs(float(self.get_parameter("rotate_speed").value))
        direction_sign = float(self.get_parameter("angular_direction_sign").value)
        self._rotate_cmd = math.copysign(rotate_speed, step_angle) * direction_sign
        self._rotate_end_sec = self._now_sec() + abs(step_angle) / max(rotate_speed, 1e-3)
        self._set_state("ROTATING")

        self.get_logger().info(
            "Rotating %.1f deg toward %s (x=%.3f y=%.3f)"
            % (math.degrees(step_angle), target.class_name, target.x, target.y)
        )

    def _handle_rotating(self) -> None:
        if self._now_sec() < self._rotate_end_sec:
            self._publish_cmd(0.0, self._rotate_cmd)
            return
        self._stop_robot()
        self._set_state("SETTLING")
        self.get_logger().info("Rotation step done; waiting for fresh detection")

    def _handle_settling(self) -> None:
        self._stop_robot()
        settle_time = float(self.get_parameter("settle_time_sec").value)
        if self._now_sec() - self._state_start_sec < settle_time:
            return

        target = self._fresh_target(after_sec=self._state_start_sec)
        if target is None:
            self._log_waiting_for_target()
            return
        self._align_or_drive(target)

    def _handle_driving(self) -> None:
        drive_timeout = float(self.get_parameter("drive_target_timeout_sec").value)
        target = self._fresh_target(timeout_sec=drive_timeout)
        if target is None:
            self._stop_robot()
            self._log_waiting_for_target()
            return

        desired = float(self.get_parameter("desired_distance").value)
        error = target.distance - desired
        if error <= 0.0:
            self._finish(
                True,
                "reached stop distance %.3fm from %s (distance=%.3f x=%.3f y=%.3f)"
                % (desired, target.class_name, target.distance, target.x, target.y),
            )
            return

        kp = float(self.get_parameter("linear_kp").value)
        max_forward = abs(float(self.get_parameter("max_forward_speed").value))
        min_linear = abs(float(self.get_parameter("min_linear_speed").value))
        heading_error = math.atan2(target.y, target.x)
        max_drive_heading_error = math.radians(
            abs(float(self.get_parameter("max_drive_heading_error_deg").value))
        )
        if abs(heading_error) > max_drive_heading_error:
            self._stop_robot()
            self.get_logger().info(
                "Target drifted while driving; realigning %s (x=%.3f y=%.3f error=%.1f deg)"
                % (target.class_name, target.x, target.y, math.degrees(heading_error))
            )
            self._begin_rotation(target)
            return

        linear = min(kp * error, max_forward)
        if linear < min_linear:
            linear = min_linear
        angular_kp = float(self.get_parameter("drive_angular_kp").value)
        max_angular = abs(float(self.get_parameter("max_drive_angular_speed").value))
        direction_sign = float(self.get_parameter("angular_direction_sign").value)
        angular = self._clamp(
            angular_kp * heading_error * direction_sign,
            -max_angular,
            max_angular,
        )
        self._publish_cmd(linear, angular)

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _stop_robot(self) -> None:
        if rclpy.ok():
            self._publish_cmd(0.0, 0.0)

    def _finish(self, success: bool, message: str) -> None:
        self._active = False
        self._set_state("IDLE")
        self._stop_robot()
        if success:
            self.get_logger().info("Approach complete: %s" % message)
        else:
            self.get_logger().warn("Approach aborted: %s" % message)

    def destroy_node(self):
        self._stop_robot()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApproachController()
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
