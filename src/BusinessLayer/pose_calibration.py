from pyniryo import *
import time
import keyboard

# List over robot IPs
robot_IPs = ["169.254.200.200", "169.254.200.201"]

# Connect to the robot
robot_0 = NiryoRobot(robot_IPs[0])
robot_1 = NiryoRobot(robot_IPs[1])

# Calibrate the motors
print("Calibrating...")
robot_0.calibrate_auto()
robot_1.calibrate_auto()

robot_0.set_learning_mode(True)
robot_1.set_learning_mode(True)

for workspace in robot_0.get_workspace_list():
    robot_0.delete_workspace(workspace)

for workspace in robot_1.get_workspace_list():
    robot_1.delete_workspace(workspace)

while True:
    if (keyboard.is_pressed('k')):
        raw_pose_0 = robot_0.get_pose()
        raw_pose_1 = robot_1.get_pose()
        point_0 = [raw_pose_0.x, raw_pose_0.y, raw_pose_0.z, raw_pose_0.roll, raw_pose_0.pitch, raw_pose_0.yaw]
        point_1 = [raw_pose_1.x, raw_pose_1.y, raw_pose_1.z, raw_pose_1.roll, raw_pose_1.pitch, raw_pose_1.yaw]
        print("Robot 0: " + str(point_0))
        print("Robot 1: " + str(point_1))
        time.sleep(2)
