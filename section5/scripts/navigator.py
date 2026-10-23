#!/usr/bin/env python3
"""A* planning, cubic spline smoothing, and differential-flatness tracking."""
import numpy as np
import rclpy
from scipy.interpolate import splrep, splev
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from astar import AStar


class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__("navigator")
        self.kp_heading = 2.0
        self.kp = 2.0
        self.kd = 2.0
        self.v_desired = 0.15
        self.spline_alpha = 0.005
        self.V_PREV_THRESH = 1e-4
        self.v_prev = 0.0
        self.t_prev = 0.0

    def compute_heading_control(self, state, goal):
        return TurtleBotControl(
            v=0.0, omega=float(self.kp_heading * wrap_angle(goal.theta - state.theta)))

    def compute_trajectory_tracking_control(self, state, plan, t):
        dt = max(0.0, t - self.t_prev)
        sample_t = np.clip(t, 0.0, plan.duration)
        xd, xd_dot, xd_ddot = [float(splev(sample_t, plan.path_x_spline, der=i))
                                for i in range(3)]
        yd, yd_dot, yd_ddot = [float(splev(sample_t, plan.path_y_spline, der=i))
                                for i in range(3)]
        # Avoid the singular inverse of the flatness map at zero speed.
        v = self.v_prev
        if abs(v) < self.V_PREV_THRESH:
            v = np.copysign(self.V_PREV_THRESH, v if v else 1.0)
        c, s = np.cos(state.theta), np.sin(state.theta)
        ux = xd_ddot + self.kp * (xd - state.x) + self.kd * (xd_dot - v * c)
        uy = yd_ddot + self.kp * (yd - state.y) + self.kd * (yd_dot - v * s)
        acceleration = c * ux + s * uy
        omega = (-s * ux + c * uy) / v
        v_command = v + acceleration * dt
        # BaseController clips the outgoing command. Remember that applied speed.
        self.v_prev = float(min(self.v_max, max(-self.v_max, v_command)))
        self.t_prev = t
        return TurtleBotControl(v=float(v_command), omega=float(omega))

    def compute_trajectory_plan(self, state, goal, occupancy, resolution, horizon):
        start = np.array([state.x, state.y])
        target = np.array([goal.x, goal.y])
        lower, upper = start - horizon, start + horizon
        # The library's occupancy checker does not reject points outside its map.
        # Restrict the search to actual map cells as well as the planning horizon.
        if hasattr(occupancy, "origin_xy"):
            lower = np.maximum(lower, occupancy.origin_xy)
            upper = np.minimum(upper, occupancy.origin_xy +
                               (occupancy.size_xy - 1) * occupancy.resolution)
        astar = AStar(tuple(lower), tuple(upper), tuple(start), tuple(target),
                      occupancy, resolution)
        if not astar.solve() or len(astar.path) < 4:
            return None
        self.v_prev = 0.0
        self.t_prev = 0.0
        path = np.asarray(astar.path)
        ts = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1)
                                  / self.v_desired)]
        # Tighten smoothing if dense samples intersect occupied space.
        for alpha in (self.spline_alpha, 0.001, 0.0):
            sx = splrep(ts, path[:, 0], k=3, s=alpha)
            sy = splrep(ts, path[:, 1], k=3, s=alpha)
            check_t = np.linspace(0.0, ts[-1], max(1000, 20 * len(path)))
            samples = np.column_stack((splev(check_t, sx), splev(check_t, sy)))
            if all(astar.is_free(point) for point in samples):
                return TrajectoryPlan(path=path, path_x_spline=sx,
                                      path_y_spline=sy, duration=float(ts[-1]))
        self.get_logger().warn("Spline samples intersect obstacles; rejecting plan")
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
