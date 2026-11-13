#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState
from std_msgs.msg import Bool
from scipy.signal import convolve2d


# this ROS2 node is responsible for autonomous frontier exploration
# uses the occupancy map and robot position to identify unexplored regions and sends navigation goals to the navigator
class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.occupancy = None
        self.state = None
        self.navigating = False
        self.finished = False
        self.frontier_candidates = []
        self.current_frontier = None
        self.failed_frontiers = []
        self.exclusion_radius = 0.5

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.state_sub = self.create_subscription(
            TurtleBotState,
            '/state',
            self.state_callback,
            10
        )

        # Subscribe to navigation results.
        # The callback is triggered when the navigator reports
        # whether a navigation goal succeeded or failed.
        self.nav_success_sub = self.create_subscription(
            Bool,
            '/nav_success',
            self.nav_success_callback,
            10
        )

        # Periodically checks whether the robot is ready for a new goal.
        # Frontier selection will only occur when the robot is not navigating.
        self.exploration_timer = self.create_timer(
            1.0,
            self.exploration_callback
        )

        # Publishes selected frontier positions as navigation goals.
        # The navigator receives these goals on /cmd_nav and attempts
        # to plan a collision-free trajectory to each destination.
        self.goal_pub = self.create_publisher(
            TurtleBotState,
            '/cmd_nav',
            10
        )

    # executes whenever a new occupancy grid is received from /map
    # converts the OccupancyGrid message into a StochOccupancyGrid2d object and stores it
    # for use by the frontier exploration algorithm
    # Does not select or publish a new navigation goal
    def map_callback(self, msg: OccupancyGrid):
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([
                msg.info.origin.position.x,
                msg.info.origin.position.y
            ]),
            window_size=9,
            probs=msg.data
        )

    # Executes whenever the robot publishes an updated state.
    # Stores the latest position and orientation so the exploration
    # algorithm can calculate distances to candidate frontiers.
    def state_callback(self, msg: TurtleBotState):
        self.state = msg

    # Called when the navigator reports the result of a goal.
    # Marks the robot as ready for another goal and records
    # whether the previous navigation attempt succeeded.
    def nav_success_callback(self, msg: Bool):
        # this function receives a notification when the navigation has finished.
        # when the function begins executing, self.navigating should still be true
        if not self.navigating:
            return

        self.navigating = False

        if msg.data:
            self.get_logger().info("Navigation succeeded")

            # The robot has reached a new location.
            # Recalculate frontiers using the latest map.
            self.frontier_candidates = []

        else:
            self.get_logger().info("Navigation failed")
            # Remember this destination so we don't attempt it again.
            self.failed_frontiers.append(self.current_frontier.copy())

            if len(self.frontier_candidates) > 0:
                distances = np.linalg.norm(
                    self.frontier_candidates - self.current_frontier,
                    axis=1
                )

                self.frontier_candidates = self.frontier_candidates[
                    distances > self.exclusion_radius
                    ]

            if len(self.frontier_candidates) > 0:
                self.publish_next_frontier()
            else:
                self.get_logger().warn("No remaining frontier candidates")

    # Checks whether the explorer is ready to select another frontier.
    # Waits until both the map and robot state are available.
    # Does not interrupt an ongoing navigation attempt.
    def exploration_callback(self):

        if self.navigating or self.finished:
            return

        if self.occupancy is None or self.state is None:
            return

        if len(self.frontier_candidates) == 0:

            frontier_states = self.find_frontiers()

            # Exclude newly generated candidates near failed destinations.
            for failed_frontier in self.failed_frontiers:

                if len(frontier_states) == 0:
                    break

                distances = np.linalg.norm(
                    frontier_states - failed_frontier,
                    axis=1
                )

                frontier_states = frontier_states[
                    distances > self.exclusion_radius
                    ]

            if len(frontier_states) == 0:
                self.finished = True
                self.get_logger().info("No valid frontiers remaining")
                return

            current_position = np.array([
                self.state.x,
                self.state.y
            ])

            distances = np.linalg.norm(
                frontier_states - current_position,
                axis=1
            )

            sorted_indices = np.argsort(distances)

            self.frontier_candidates = frontier_states[sorted_indices]

        self.publish_next_frontier()

    # Identifies candidate frontiers using the latest occupancy grid.
    # Returns their coordinates in the map's world reference frame.
    def find_frontiers(self):

        occupancy = self.occupancy

        self.get_logger().info(
            f"Map shape: {occupancy.probs.shape}, "
            f"min: {np.min(occupancy.probs)}, "
            f"max: {np.max(occupancy.probs)}"
        )

        window_size = 13

        # Classify occupancy grid cells
        unknown = occupancy.probs == -1
        occupied = occupancy.probs >= 50
        unoccupied = (occupancy.probs >= 0) & (occupancy.probs < 50)

        # Create a neighborhood search kernel
        kernel = np.ones((window_size, window_size))

        # Require at least 20% unknown cells
        unknown_count = convolve2d(unknown, kernel, mode='same')
        unknown_fraction = unknown_count / (window_size ** 2)
        valid_unknown = unknown_fraction >= 0.20

        # Reject neighborhoods containing occupied cells
        occupied_count = convolve2d(occupied, kernel, mode='same')
        valid_occupied = occupied_count == 0

        # Require at least 30% free cells
        unoccupied_count = convolve2d(unoccupied, kernel, mode='same')
        unoccupied_fraction = unoccupied_count / (window_size ** 2)
        valid_unoccupied = unoccupied_fraction >= 0.30

        # Combine the three conditions
        valid_frontiers = (
                valid_unknown &
                valid_occupied &
                valid_unoccupied &
                unoccupied
        )

        # Convert valid grid indices into world coordinates
        rows, cols = np.where(valid_frontiers)
        grid_xy = np.column_stack((cols, rows))

        frontier_states = occupancy.grid2state(grid_xy)

        return frontier_states

    # Sends the next candidate frontier to the navigator.
    # Removes the selected candidate from the list so that
    # a failed navigation attempt can move to the next one.
    def publish_next_frontier(self):

        if len(self.frontier_candidates) == 0:
            return

        frontier = self.frontier_candidates[0]
        self.current_frontier = frontier.copy()
        self.frontier_candidates = self.frontier_candidates[1:]

        goal = TurtleBotState()
        goal.x = float(frontier[0])
        goal.y = float(frontier[1])
        goal.theta = 0.0

        self.navigating = True
        self.goal_pub.publish(goal)


# Initializes ROS2, creates the frontier exploration node,
# and keeps it running so it can process incoming messages.
def main(args=None):
    rclpy.init(args=args)

    explorer = FrontierExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

