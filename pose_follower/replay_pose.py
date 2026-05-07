import time
import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
from rclpy.time import Time
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped, Transform
import tf2_geometry_msgs


# Conservative max velocities — joint moves at this speed, duration set by distance.
JOINT_MAX_VEL = {
    'joint_lift': 0.1,            # m/s
    'wrist_extension': 0.06,       # m/s
    'joint_wrist_yaw': 1.5,        # rad/s
    'joint_wrist_pitch': 1.5,      # rad/s
    'joint_wrist_roll': 1.5,       # rad/s
}
BASE_MAX_VEL = 0.15                # m/s
MIN_DURATION = 2.0                 # seconds


def _duration_for(joint_names, start_pos, target_pos):
    durations = [abs(t - s) / JOINT_MAX_VEL.get(n, 0.1)
                 for n, s, t in zip(joint_names, start_pos, target_pos)]
    return max(max(durations) if durations else 0.0, MIN_DURATION)


class PoseReplayer():
    def __init__(self, node):
        self.node = node
        self.tf_buffer = node.tf_buffer
        self.trajectory_client = node.trajectory_client

    def _send(self, joint_names, start_pos, target_pos, duration=None):
        if duration is None:
            duration = _duration_for(joint_names, start_pos, target_pos)
        points = []
        if start_pos is not None:
            sp = JointTrajectoryPoint()
            sp.time_from_start = Duration(seconds=0.0).to_msg()
            sp.positions = start_pos
            points.append(sp)
        ep = JointTrajectoryPoint()
        ep.time_from_start = Duration(seconds=duration).to_msg()
        ep.positions = target_pos
        points.append(ep)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points
        goal.trajectory.header.frame_id = 'base_link'

        task = self.trajectory_client.send_goal_async(goal)
        while not task.done():
            time.sleep(0.05)
        if not task.result().accepted:
            self.node.get_logger().error(f"Goal rejected for {joint_names}")
            return
        result_future = task.result().get_result_async()
        while not result_future.done():
            time.sleep(0.05)
        time.sleep(0.2)

    def _send_base(self, delta_x, duration=None):
        if duration is None:
            duration = max(abs(delta_x) / BASE_MAX_VEL, MIN_DURATION)

        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', Time(),
                timeout=Duration(seconds=1.0),
            )
            cur_x = tf.transform.translation.x
            cur_y = tf.transform.translation.y
            cur_rot = tf.transform.rotation
        except TransformException as e:
            self.node.get_logger().error(f"Cannot read odom: {e}")
            return

        target_x_odom = cur_x + delta_x

        sp = MultiDOFJointTrajectoryPoint()
        sp.time_from_start = Duration(seconds=0.0).to_msg()
        t0 = Transform()
        t0.translation.x = cur_x
        t0.translation.y = cur_y
        t0.rotation = cur_rot
        sp.transforms = [t0]

        ep = MultiDOFJointTrajectoryPoint()
        ep.time_from_start = Duration(seconds=duration).to_msg()
        t1 = Transform()
        t1.translation.x = target_x_odom
        t1.translation.y = cur_y
        t1.rotation = cur_rot
        ep.transforms = [t1]

        goal = FollowJointTrajectory.Goal()
        goal.multi_dof_trajectory.joint_names = ['position']
        goal.multi_dof_trajectory.points = [sp, ep]

        task = self.trajectory_client.send_goal_async(goal)
        while not task.done():
            time.sleep(0.05)
        if not task.result().accepted:
            self.node.get_logger().error("Base goal rejected")
            return
        result_future = task.result().get_result_async()
        while not result_future.done():
            time.sleep(0.05)
        time.sleep(0.2)

    def replay_poses(self, sequence):
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
                    timeout=Duration(seconds=1.0),
                )
            except TransformException as e:
                self.node.get_logger().error(f"TF transform failed: {e}")
                continue

            self._send_base(pose_to_base.pose.position.x)

            js = self.node.latest_joint_state
            def cur(name):
                try: return float(js.position[js.name.index(name)])
                except ValueError: return 0.0

            self._send(
                ['joint_lift', 'wrist_extension',
                 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'],
                [
                    cur('joint_lift'),
                    sum(cur(f'joint_arm_l{i}') for i in range(4)),
                    cur('joint_wrist_yaw'),
                    cur('joint_wrist_pitch'),
                    cur('joint_wrist_roll'),
                ],
                [
                    point['joint_lift'],
                    point['wrist_extension'],
                    point['wrist_yaw'],
                    point['wrist_pitch'],
                    point['wrist_roll'],
                ],
            )
