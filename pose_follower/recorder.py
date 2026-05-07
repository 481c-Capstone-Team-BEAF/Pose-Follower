import rclpy
from replay_pose import PoseReplayer
from record_pose import PoseRecorder
import time
import os
import json

'''
Run these commands FIRST:
ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory broadcast_odom_tf:=True
ros2 launch stretch_core d435i_high_resolution.launch.py
ros2 launch stretch_core stretch_aruco.launch.py
ros2 run stretch_core keyboard_teleop

For gamepad instead of keyboard:
ros2 launch stretch_core stretch_driver.launch.py mode:=gamepad broadcast_odom_tf:=True
ros2 launch stretch_core d435i_high_resolution.launch.py initial_reset:=false
ros2 launch stretch_core stretch_aruco.launch.py

Before replaying, switch to trajectory mode:
ros2 service call /switch_to_trajectory_mode std_srvs/srv/Trigger
'''

import sys, select, termios, tty, threading


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_DIR = os.path.join(PROJECT_ROOT, 'pose_follower_save', 'config')
POSES_FILE = os.path.join(CONFIG_DIR, 'poses.json')


def load_poses():
    if not os.path.exists(POSES_FILE):
        return {}
    try:
        with open(POSES_FILE, 'r') as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def save_poses(pose_dict):
    os.makedirs(CONFIG_DIR, exist_ok=True)
    with open(POSES_FILE, 'w') as f:
        json.dump(pose_dict, f, indent=2)


class KeyboardWatcher:
    def __init__(self):
        self.record_requested = False
        self.end_requested = False
        self._running = True
        self._old_settings = termios.tcgetattr(sys.stdin)
        new_settings = termios.tcgetattr(sys.stdin)
        new_settings[3] &= ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(sys.stdin, termios.TCSANOW, new_settings)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch == '1':
                    self.record_requested = True
                elif ch == '0':
                    self.end_requested = True

    def stop(self):
        self._running = False
        self._thread.join(timeout=0.5)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)


def main():
    pose_recorder = PoseRecorder()
    pose_replayer = PoseReplayer(pose_recorder)

    pose_dict = load_poses()
    if pose_dict:
        print(f"Loaded {len(pose_dict)} pose sequence(s) from {POSES_FILE}")

    while True:

        selection = None
        while True:
            try:
                selection = int(input("What would you like to do?\n- [1] Begin collecting poses\n- [2] Follow pose sequence\n- [3] Exit\n"))
                break
            except Exception as e:
                print(f"Please provide valid input\nEncountered {e}")

        if selection == 3: break
        elif selection == 2:
            if len(pose_dict) == 0:
                print("There are no recorded poses")
                continue
            pose_sequence = input(f"The following pose sequences are available:\n{list(pose_dict.keys())}\nPlease select one here: ")
            while pose_sequence not in pose_dict.keys():
                pose_sequence = input("Invalid pose sequence, please select again: ")
            pose_replayer.replay_poses(pose_dict[pose_sequence])
        elif selection == 1:
            print("Please move the robot, and press [1] to record, [0] to end")
            watcher = KeyboardWatcher()
            curr_points = []
            while not watcher.end_requested:
                if watcher.record_requested:
                    watcher.record_requested = False
                    new_point = pose_recorder.record()
                    if new_point is None:
                        print("Ensure ArUco marker is visible before recording pose")
                        continue
                    else:
                        curr_points.append(new_point)
                        print(f"Recorded pose {len(curr_points)}")
                time.sleep(0.05)
            watcher.stop()

            if len(curr_points) == 0:
                print("No poses recorded")
                continue

            name = input("Please provide a name for this pose sequence: ")
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
            pose_dict[name] = curr_points
            save_poses(pose_dict)
            print(f"Saved new pose sequence with name <{name}>")

    pose_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
