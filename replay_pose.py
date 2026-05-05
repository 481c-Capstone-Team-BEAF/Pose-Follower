import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

import hello_helpers.hello_misc as hm


class PoseReplayer():
    def __init__(self, node):
        self.node = node
        self.tf_buffer = node.tf_buffer
        self.trajectory_client = node.trajectory_client

    def replay_poses(self, sequence):
        try:
            tf_0 = self.tf_buffer.lookup_transform(
                'odom', 'base_link', Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as ex:
            self.node.get_logger().error(f"Cannot read base position: {ex}")
            return
        
        start_x = tf_0.transform.translation.x
        
        for point in sequence:
            pose = PoseStamped()
            pose.header.frame_id = point['frame_id']
            pose.header.stamp = Time().to_msg()
            pose.pose.position.x = point['position']['x']
            pose.pose.position.y = point['position']['y']
            pose.pose.position.z = point['position']['z']
            pose.pose.orientation.x = point['orientation']['x']
            pose.pose.orientation.y = point['orientation']['y']
            pose.pose.orientation.z = point['orientation']['z']
            pose.pose.orientation.w = point['orientation']['w']

            try:
                pose_to_base = self.tf_buffer.transform(
                    pose, 'base_link',
                    timeout=Duration(seconds=1.0)
                )
            except TransformException as e:
                self.node.get_logger().error(f"TF transform failed: {e}")
                continue

            move_to_point = JointTrajectoryPoint()
            move_to_point.time_from_start = Duration(seconds=4.0).to_msg()
            move_to_point.positions = [
                pose_to_base.pose.position.x - start_x,
                pose_to_base.pose.position.z,
                -pose_to_base.pose.position.y,
                point['wrist_yaw'],
                point['wrist_pitch'],
                point['wrist_roll']
            ]

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = ['translate_mobile_base', 'joint_lift', 'joint_arm',
                       'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
            trajectory_goal.trajectory.points = [move_to_point]
            trajectory_goal.trajectory.header.frame_id = 'base_link'
            
            task = self.trajectory_client.send_goal_async(trajectory_goal)
            rclpy.spin_until_future_complete(self.node, task)
            task_result = task.result()
            if task_result.accepted:
                result_future = task_result.get_result_async()
                rclpy.spin_until_future_complete(self.node, result_future)

            self.node.move_to_pose(
                {'joint_gripper_finger_right': point['gripper_finger']},
                blocking=True,
            )
