import rclpy
from replay_pose import PoseReplayer
from record_pose import PoseRecorder
from pynput import keyboard
import time

'''
Need to request enter user-control state:
    - "What would you like to do?"
        - [1] Begin collecting poses
        - [2] Follow pose sequence
        - [3] Exit
    If select "Begin collecting poses":
        - "Please move the robot, and press [enter] to record pose, or [escape] to end"
        If select [escape]
            - Please write a name for this pose sequence: {}
            Pose sequence is a list, which is stored in dictionary mapped to the typed name
    If select "Follow pose sequence":
        - "The following pose sequences are available, please type one in:"
        The user will write the desired pose sequence name
Repeat this in a while loop, where gamepad_teleop is able to run the entire time
'''

'''
Run these commands FIRST:
ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory
ros2 run stretch_core detect_aruco_markers
ros2 run stretch_core stretch_gamepad_teleop
'''

class KeyboardWatcher:
    def __init__(self):
        self.record_requested = False
        self.end_requested = False
        self._listener = keyboard.Listener(on_press=self._on_press)
        self._listener.start()

    def _on_press(self, key):
        if key == keyboard.Key.enter:
            self.record_requested = True
        elif key == keyboard.Key.esc:
            self.end_requested = True

    def stop(self):
        self._listener.stop()


def main():
    rclpy.init()

    pose_recorder = PoseRecorder()
    pose_replayer = PoseReplayer()
    pose_dict = {}

    while True:

        selection = None
        while True:
            try:
                selection = int(input("What would you like to do?\n- [1] Begin collecting poses\n- [2] Follow pose sequence\n- [3] Exit"))
                break
            except Exception as e:
                print(f"Please provide valid input\nEncountered {e}")
            
        if selection == 3: break
        elif selection == 2:
            if len(pose_dict) == 0:
                print("There are no recorded poses")
                continue
            pose_sequence = input(f"The following pose sequences are available:\n{pose_dict.keys()}\nPlease select one here: ")
            while pose_sequence not in pose_dict.keys():
                pose_sequence = input("Invalid pose sequence, please select again: ")
            selected_pose_sequence = pose_dict[pose_sequence]
            #use pose sequence to move robot to position
            pose_replayer.replay_poses(selected_pose_sequence)
        elif selection == 1:
            print("Please move the robot, and press [enter] to record pose, or [escape] to end")
            #Wait for [enter] or [escape] key and perform action upon completion
            watcher = KeyboardWatcher()
            curr_points = []
            while not watcher.end_requested:
                if watcher.record_requested:
                    watcher.record_requested = False
                    new_point = pose_recorder.record()
                    if (new_point is None):
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
            if (name in pose_dict.keys()):
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
            print(f"Saved new pose sequence with name <{name}>")
        
    pose_recorder.destroy_node()
    pose_replayer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
