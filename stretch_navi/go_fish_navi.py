import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult, TaskResult


#Need to figure out how to get the gamebooard location
# WAYPOINTS is what i called the file

class GoFishNavi(Node):
    def __init__(self):
        super().__init__("go_fish_navi")

        self.navigator = BasicNavigator()
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.stop_sub = self.create_subscription(
            Bool,
            "stop_navigation",
            self.stop_callback,
            10,
        )

        self.stop_navigation = False

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

    def stop_callback(self, msg):
        self.stop_navigation = msg.data
        if self.stop_navigation:
            self.get_logger().info("Stop signal received.")

    def make_pose(self, waypoint_name):
        wp = WAYPOINTS[waypoint_name]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        pose.pose.position.x = wp["x"]
        pose.pose.position.y = wp["y"]
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = wp["qz"]
        pose.pose.orientation.w = wp["qw"]

        return pose

    def navigate_to_waypoint(self, waypoint_name):
        self.stop_navigation = False

        goal_pose = self.make_pose(waypoint_name)

        self.get_logger().info(f"Sending robot to {waypoint_name}...")
        self.navigator.goToPose(goal_pose)

        while rclpy.ok() and not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.stop_navigation:
                self.get_logger().info("Canceling navigation...")
                self.navigator.cancelTask()
                self.stop_robot()
                return False

            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f} m"
                )

            time.sleep(0.3)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {waypoint_name}.")
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f"Navigation to {waypoint_name} was canceled.")
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error(f"Navigation to {waypoint_name} failed.")
            return False
        else:
            self.get_logger().error(f"Unknown navigation result: {result}")
            return False

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run_gameboard_to_user(self):
        success = self.navigate_to_waypoint("user")

        if success:
            self.get_logger().info("Robot moved from gameboard to user.")
        else:
            self.get_logger().error("Robot could not reach user.")


def main(args=None):
    rclpy.init(args=args)

    node = GoFishNavi()

    try:
        node.run_gameboard_to_user()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()