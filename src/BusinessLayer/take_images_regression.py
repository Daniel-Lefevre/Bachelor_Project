import os
import sys
import time

# Adds the project root (Bachelor_Project) to the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import cv2
import keyboard
import paramiko
from pyniryo import ConveyorDirection, NiryoRobot, uncompress_image

from resources.environment import configuration


def take_and_store_image(robot_index, counter, robot):
    img_compressed = robot.get_img_compressed()
    if img_compressed:
        # Uncompress and save the image
        img = uncompress_image(img_compressed)
        base_dir = r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment3Regression\Training_Data\Conveyor_0"
        dynamic_folder = f"Robot_{robot_index}"
        filename = str(counter) + ".jpg"

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

robot_0.move_pose(*configuration["robot_0_overview_pose"])
robot_1.move_pose(*configuration["robot_1_overview_pose"])

conveyor_id_0 = robot_0.set_conveyor()
conveyor_id_1 = robot_1.set_conveyor()

robot_0.run_conveyor(conveyor_id_0, speed=0, direction=ConveyorDirection.BACKWARD)
robot_1.run_conveyor(conveyor_id_1, speed=0, direction=ConveyorDirection.BACKWARD)


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

counter = 1

while True:
    if keyboard.is_pressed("1"):
        take_and_store_image(0, counter, robot_0)
        take_and_store_image(1, counter, robot_1)
        time.sleep(1)
        counter += 1
