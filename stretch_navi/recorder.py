import rclpy
import time
import os
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from go_fish_navi import GoFishNavi, load_poses as load_poses_navi

'''
A saved map of the environment is needed. Create one by using:
ros2 launch stretch_nav2 offline_mapping.launch.py
In a new terminal, save the map with:
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>

FOLLOW THESE STEPS TO RUN:

- ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
- In RViz, click "2D Pose Estimate" and place where the robot is. Then, press the "Startup" button in RViz
- Run the robot a bit
- run python3 recorder.py
- USAGE:
  [1] Record current pose: drive the robot to a spot with the gamepad,
      type 1, confirm with Y, then name the location. The pose is saved to the saved-poses file.
  [2] Go to recorded pose: pick a saved name, the script lazily connects
      to Nav2 (first time only) and sends the robot autonomously.
  [3] exits
'''

POSE_FILE = os.path.expanduser('~/beaf_ws/stretch_navi/stretch_saved_poses.json')


def load_poses():
    # Loads poses from a file to a dict
    # dict is of form { name : PoseStamped() object }
    if not os.path.exists(POSE_FILE):
        return {}
    with open(POSE_FILE, 'r') as f:
        raw = json.load(f)
    pose_dict = {}
    for name, d in raw.items():
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = d['x']
        ps.pose.position.y = d['y']
        ps.pose.position.z = d['z']
        ps.pose.orientation.x = d['qx']
        ps.pose.orientation.y = d['qy']
        ps.pose.orientation.z = d['qz']
        ps.pose.orientation.w = d['qw']
        pose_dict[name] = ps
    return pose_dict

def save_pose(pose_dict):
    raw = {}
    for name, ps in pose_dict.items():
        raw[name] = {
            'x': ps.pose.position.x,
            'y': ps.pose.position.y,
            'z': ps.pose.position.z,
            'qx': ps.pose.orientation.x,
            'qy': ps.pose.orientation.y,
            'qz': ps.pose.orientation.z,
            'qw': ps.pose.orientation.w,
        }
    with open(POSE_FILE, 'w') as f:
        json.dump(raw, f, indent=2)

def get_current_pose(node, tf_buffer):
    # Spin briefly so the TF buffer has fresh data
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    try:
        tf = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    except (LookupException, ExtrapolationException) as e:
        node.get_logger().error(f'TF lookup failed: {e}')
        return None
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = tf.transform.translation.x
    ps.pose.position.y = tf.transform.translation.y
    ps.pose.position.z = tf.transform.translation.z
    ps.pose.orientation = tf.transform.rotation
    return ps

def main():
    rclpy.init()
    node = Node('pose_recorder')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    pose_dict = load_poses()
    navi = None

    while True:
        selection = None
        while True:
            try:
                selection = int(input("What would you like to do?\n- [1] Record current pose\n- [2] Go to recorded pose\n- [3] Exit\n"))
                break
            except Exception as e:
                print(f"Please provide valid input\nEncountered {e}")

        if selection == 3: break
        elif selection == 2:
            if len(pose_dict) == 0:
                print("There are no recorded poses")
                continue
            pose_object = input(f"The following named locations are available:\n{list(pose_dict.keys())}\nPlease select one here: ")
            while pose_object not in pose_dict.keys():
                pose_object = input("Invalid location, please select again: ")
            if navi is None:
                print("Connecting to nav2")
                navi = GoFishNavi()
            navi.navigate_to_waypoint(pose_object)
        elif selection == 1:
            while True:
                confirm = input("Please go to the desired location before saving. Type Y to end and save: ")
                if confirm in ('Y', 'y', 'yes', 'Yes'): break
                else: continue

            #save to pose_object here
            pose_object = get_current_pose(node, tf_buffer)
            if pose_object is None:
                print("Could not read current pose, try again.")
                continue
            
            name = input("Please provide a name for this location: ")
            if name in pose_dict.keys():
                while True:
                    confirm = input("This sequence name already exists, would you like to overwrite? [Y/n]")
                    if confirm in ('Y', 'y', 'yes', 'Yes'): break
                    if confirm in ('N', 'n', 'no', 'No'):
                        name = input("Please provide a new name for this pose sequence: ")
                        if name in pose_dict.keys(): continue
                        break
                    else:
                        print("Invalid input")
            pose_dict[name] = pose_object
            save_pose(pose_dict)
            if navi is not None:
                navi.waypoints = load_poses_navi()
            print(f"Saved new location with name <{name}>")