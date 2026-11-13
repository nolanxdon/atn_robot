#!/usr/bin/env python3

import rclpy
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl
from std_msgs.msg import Bool


class PerceptionController(BaseController):
    def __init__(self):
        super().__init__("perception_controller")
        self.declare_parameter("active", True)
        self.stop_start_time = None
        # after resuming, ignore detections for this long so the robot can
        # rotate the stop sign out of view instead of re-triggering on it
        self.detection_cooldown = 3.0  # seconds
        self.resume_time = None

        self.detector_sub = self.create_subscription(
            Bool, "/detector_bool", self.detector_callback, 10
        )

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value

    def detector_callback(self, msg: Bool) -> None:
        if not msg.data or not self.active:
            return
        if self.resume_time is not None:
            since_resume = (self.get_clock().now() - self.resume_time).nanoseconds * 1e-9
            if since_resume < self.detection_cooldown:
                return
        self.get_logger().info("Stop sign detected: stopping for 5 seconds")
        self.set_parameters([rclpy.Parameter("active", value=False)])

    def compute_control(self) -> TurtleBotControl:
        control = TurtleBotControl()

        if self.active:
            self.stop_start_time = None
            control.omega = 0.5
            return control

        # active is False: stop for 5 seconds, then resume and reset active
        now = self.get_clock().now()
        if self.stop_start_time is None:
            self.stop_start_time = now

        if (now - self.stop_start_time).nanoseconds * 1e-9 >= 5.0:
            self.set_parameters([rclpy.Parameter("active", value=True)])
            self.stop_start_time = None
            self.resume_time = now
            control.omega = 0.5
        else:
            control.omega = 0.0

        return control


if __name__ == "__main__":
    rclpy.init()
    controller = PerceptionController()
    rclpy.spin(controller)
    rclpy.shutdown()
