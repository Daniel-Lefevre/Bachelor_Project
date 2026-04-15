import os
import sys
import time

# Adds the project root (Bachelor_Project) to the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import cv2
import keyboard
import paramiko
from pyniryo import NiryoRobot, uncompress_image

from resources.environment import configuration


def take_and_store_image(robot_index, label, robots, filename):
    # Take in image
    img_compressed = robots[robot_index].get_img_compressed()
    if img_compressed:
        # Uncompress and save the image
        img = uncompress_image(img_compressed)
        base_dir = r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment1OpenSetRecognision\Training_Data"
        dynamic_folder = label
        filename = str(filename) + ".jpg"

        # Combine base directory, dynamic folder, and filename
        full_path = os.path.join(base_dir, dynamic_folder, filename)

        # Save the image
        cv2.imwrite(full_path, img)
        print("Saved image")

    else:
        print("Could not take image")
    time.sleep(1)


# List over robot IPs
robot_IPs = ["169.254.200.200", "169.254.200.201"]

# Connect to the robot
robot_0 = NiryoRobot(robot_IPs[0])
robot_1 = NiryoRobot(robot_IPs[1])

robots = [robot_0, robot_1]

# Calibrate the motors
print("Calibrating...")
robot_0.calibrate_auto()
robot_1.calibrate_auto()

place_conveyor_0, observation_0, observation_pose_storage_0, standby_position_0, observation_pose_conveyor_0 = configuration["positions"][0]
place_conveyor_1, observation_1, observation_pose_storage_1, standby_position_1, observation_pose_conveyor_1 = configuration["positions"][1]


robot_0.move_pose(*observation_pose_conveyor_0)
robot_1.move_pose(*observation_pose_conveyor_1)


def _enable_camera(ip: str) -> bool:
    output = ""
    setup_file = "/home/niryo/catkin_ws/install/release/ned2/setup.bash"
    camera_topic = "/niryo_robot_vision/compressed_video_stream"
    # Connect to the robot via SSH
    with paramiko.SSHClient() as client:
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(ip, username="niryo", password="robotics", timeout=3)
            full_cmd = (
                f"source {setup_file} && "  # Source the setup file
                f"rosservice call /niryo_robot_vision/start_stop_video_streaming '{{value: true}}' && "  # Enable camera
                f"timeout 2s rostopic hz {camera_topic}"  # Run the ros topic for 2 seconds to see if the camera publishes anything (saving the output in the next commando)
            )
            _, stdout, _ = client.exec_command(full_cmd)
            # Wait for the command to finish (because we have a 2 second timer in the last command)
            stdout.channel.recv_exit_status()
            output = stdout.read().decode()
        except Exception as e:
            print(f"SSH to check camera failed: {e}")
            return False

    return "average rate" in output


for ip in robot_IPs:
    print(_enable_camera(ip))

labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "No_Object", "Unidentified_Object"]
label_counter = {"Blue_Circle": 100, "Blue_Square": 100, "Red_Circle": 100, "Red_Square": 100, "Green_Circle": 100, "Green_Square": 100, "No_Object": 100, "Unidentified_Object": 125}

while True:
    # Robot 0 Blue Circle
    if keyboard.is_pressed("1"):
        label = labels[0]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 Blue Square
    if keyboard.is_pressed("2"):
        label = labels[1]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 Red Circle
    if keyboard.is_pressed("3"):
        label = labels[2]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 Red Sqaure
    if keyboard.is_pressed("4"):
        label = labels[3]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 Green Circle
    if keyboard.is_pressed("5"):
        label = labels[4]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 Green Square
    if keyboard.is_pressed("6"):
        label = labels[5]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 No Object
    if keyboard.is_pressed("7"):
        label = labels[6]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 0 Unidentified object
    if keyboard.is_pressed("8"):
        label = labels[7]
        label_counter[label] += 1
        take_and_store_image(0, label, robots, label_counter[label])

    # Robot 1 Blue Circle
    if keyboard.is_pressed("q"):
        label = labels[0]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 Blue Square
    if keyboard.is_pressed("w"):
        label = labels[1]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 Red Circle
    if keyboard.is_pressed("e"):
        label = labels[2]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 Red Sqaure
    if keyboard.is_pressed("r"):
        label = labels[3]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 Green Circle
    if keyboard.is_pressed("t"):
        label = labels[4]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 Green Square
    if keyboard.is_pressed("y"):
        label = labels[5]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 No Object
    if keyboard.is_pressed("u"):
        label = labels[6]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])

    # Robot 1 Unidentified object
    if keyboard.is_pressed("i"):
        label = labels[7]
        label_counter[label] += 1
        take_and_store_image(1, label, robots, label_counter[label])
