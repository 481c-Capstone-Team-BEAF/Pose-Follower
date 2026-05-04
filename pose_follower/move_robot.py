import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import sys
import termios
import threading

class FrameListener(node):
    def __init__(self):
        super().__init__('stretch_tf_listener')
        self.declare_parameter('target_frame', 'link_grasp_center')
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []

        self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        pass

    def save_waypoint(self);
        from_frame_rel = self.target_frame
        to_frame_rel = 'fk_link_mast'
        try:
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                Time()
            )
            pose = {
                'x' : trans.transform.translation.x,
                'y' : trans.transform.translation.y,
                'z' : trans.transform.translation.z,
                'qx' : trans.transform.translation.x,
                'qy' : trans.transform.translation.y,
                'qz' : trans.transform.translation.z,
                'qw' : trans.transform.translation.w
            }
            self.waypoints.append(pose)
            self.get_logger().info(f'Saved waypoint {len(self.waypoint)}: {pose}')
        except TransformException as ex:
            self.get_logger().warn(f'Could not get pose: {ex}')

    def save_to_file(self, filename = 'fishing_demo.yaml'):
        with open(filename, 'w') as f:
            yaml.dump(self.waypoints, f)
        self.get_logger.info(f'Saved {len(self.waypoints)} waypoints to {filename}')