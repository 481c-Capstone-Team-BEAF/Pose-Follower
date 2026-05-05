import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.duration import Duration

import hello_helpers.hello_misc as hm

class PoseRecorder(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(
            self, 'pose_recorder', 'pose_recorder',
            wait_for_first_pointcloud=False,
        )

        self.latest_joint_state = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.parent_frame = 'target_marker'
        self.child_frame  = 'link_grasp_center'

        self.create_subscription(
            JointState, '/stretch/joint_states',
            self.get_joint_state, 1,
        )

    def get_joint_state(self, msg):
        self.latest_joint_state = msg

    def record(self):
        
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not look up {self.child_frame} in {self.parent_frame}: {ex}"
            )
            return None

        if self.latest_joint_state is None:
            self.get_logger().warn("Joint state or odom not yet received; cannot record.")
            return None

        js = self.latest_joint_state
        try:
            wrist_yaw = js.position[js.name.index('joint_wrist_yaw')]
            wrist_pitch = js.position[js.name.index('joint_wrist_pitch')]
            wrist_roll = js.position[js.name.index('joint_wrist_roll')]
            gripper_finger = js.position[js.name.index('joint_gripper_finger_right')]

        except ValueError as e:
            self.get_logger().error(f"Expected joint missing from /stretch/joint_states: {e}")
            return None

        return {
            'frame_id': self.parent_frame,
            'position': {
                'x': tf.transform.translation.x,
                'y': tf.transform.translation.y,
                'z': tf.transform.translation.z
            },
            'orientation': {
                'x': tf.transform.rotation.x,
                'y': tf.transform.rotation.y,
                'z': tf.transform.rotation.z,
                'w': tf.transform.rotation.w
            },
            'wrist_yaw': float(wrist_yaw),
            'wrist_pitch': float(wrist_pitch),
            'wrist_roll': float(wrist_roll),
            'gripper_finger': float(gripper_finger)
        }