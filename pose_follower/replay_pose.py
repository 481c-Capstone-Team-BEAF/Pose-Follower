import time
from rclpy.duration import Duration
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped, Transform
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped for tf_buffer.transform)


JOINT_MAX_VEL = {
    'joint_lift': 0.1, 'wrist_extension': 0.06,
    'joint_wrist_yaw': 1.5, 'joint_wrist_pitch': 1.5, 'joint_wrist_roll': 1.5,
}
BASE_MAX_VEL = 0.15
MIN_DURATION = 2.0


class PoseReplayer():
    def __init__(self, node):
        self.node = node
        self.tf_buffer = node.tf_buffer
        self.trajectory_client = node.trajectory_client

    def _wait(self, goal):
        task = self.trajectory_client.send_goal_async(goal)
        while not task.done():
            time.sleep(0.05)
        if not task.result().accepted:
            self.node.get_logger().error("Goal rejected")
            return
        result = task.result().get_result_async()
        while not result.done():
            time.sleep(0.05)
        time.sleep(0.2)  # let joint_states catch up before the next goal

    def _send_joints(self, names, start, target):
        duration = max(MIN_DURATION, max(
            abs(t - s) / JOINT_MAX_VEL[n] for n, s, t in zip(names, start, target)
        ))
        sp = JointTrajectoryPoint(positions=start, time_from_start=Duration(seconds=0.0).to_msg())
        ep = JointTrajectoryPoint(positions=target, time_from_start=Duration(seconds=duration).to_msg())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [sp, ep]
        self._wait(goal)

    def _send_base(self, target_x_odom):
        tf = self.tf_buffer.lookup_transform('odom', 'base_link', Time(),
                                             timeout=Duration(seconds=1.0))
        cur_x = tf.transform.translation.x
        duration = max(MIN_DURATION, abs(target_x_odom - cur_x) / BASE_MAX_VEL)

        def waypoint(x, t_sec):
            p = MultiDOFJointTrajectoryPoint()
            p.time_from_start = Duration(seconds=t_sec).to_msg()
            tr = Transform()
            tr.translation.x = x
            tr.translation.y = tf.transform.translation.y
            tr.rotation = tf.transform.rotation
            p.transforms = [tr]
            return p

        goal = FollowJointTrajectory.Goal()
        goal.multi_dof_trajectory.joint_names = ['position']
        goal.multi_dof_trajectory.points = [waypoint(cur_x, 0.0), waypoint(target_x_odom, duration)]
        self._wait(goal)

    def replay_poses(self, sequence):
        # One marker snapshot up front: resolve every base target into odom now so
        # per-pose ArUco noise can't perturb the sequence mid-replay.
        targets = []
        for p in sequence:
            pose = PoseStamped()
            pose.header.frame_id = p['frame_id']
            pose.pose.position.x = p['position']['x']
            pose.pose.position.y = p['position']['y']
            pose.pose.position.z = p['position']['z']
            pose.pose.orientation.x = p['orientation']['x']
            pose.pose.orientation.y = p['orientation']['y']
            pose.pose.orientation.z = p['orientation']['z']
            pose.pose.orientation.w = p['orientation']['w']
            try:
                in_odom = self.tf_buffer.transform(pose, 'odom', timeout=Duration(seconds=1.0))
            except TransformException as e:
                self.node.get_logger().error(f"TF transform failed (is the marker visible?): {e}")
                return
            targets.append(in_odom.pose.position.x)

        for p, target_x in zip(sequence, targets):
            self._send_base(target_x)
            js = self.node.latest_joint_state
            def jp(n): return float(js.position[js.name.index(n)])
            self._send_joints(
                ['joint_lift', 'wrist_extension',
                 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'],
                [jp('joint_lift'), sum(jp(f'joint_arm_l{i}') for i in range(4)),
                 jp('joint_wrist_yaw'), jp('joint_wrist_pitch'), jp('joint_wrist_roll')],
                [p['joint_lift'], p['wrist_extension'],
                 p['wrist_yaw'], p['wrist_pitch'], p['wrist_roll']],
            )
