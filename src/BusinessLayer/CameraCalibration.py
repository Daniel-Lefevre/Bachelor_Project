import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(src_dir)
sys.path.append(project_root)
from pyniryo import *
from resources.environment import configuration
import time
import numpy as np
import random

# List over robot IPs
robot_IPs = ["169.254.200.200", "169.254.200.201"]

# Connect to the robot
robot_0 = NiryoRobot(robot_IPs[0])
robot_1 = NiryoRobot(robot_IPs[1])

robot_0_conveyor_workspace = "Conveyor_workspace_0"
robot_1_conveyor_workspace = "Conveyor_workspace_1"

if (robot_0.hardware_status.calibration_needed):
    robot_0.calibrate_auto()
if (robot_1.hardware_status.calibration_needed):
    robot_1.calibrate_auto()

# Conveyor
#robot_0.move_pose(*configuration["positions"][0][2])
#robot_1.move_pose(*configuration["positions"][1][2])

# Storage
robot_0.move_pose(*configuration["positions"][0][3])
robot_1.move_pose(*configuration["positions"][1][3])

for shape in [ObjectShape.SQUARE, ObjectShape.CIRCLE]:
    for color in [ObjectColor.RED, ObjectColor.BLUE, ObjectColor.GREEN]:
        sum = np.zeros((3))
        count = 0

        for i in range(100):
            brightness = random.uniform(0.5,2)
            contrast = random.uniform(0.5,1.5)
            saturation = random.uniform(0.5,2)

            robot_1.set_brightness(brightness)
            robot_1.set_contrast(contrast)
            robot_1.set_saturation(saturation)

            obj_found, object_pose, shape_ret, color_ret = robot_1.detect_object(robot_1_conveyor_workspace,
                                                                                shape=shape,
                                                                                color=color
                                                                                )
            print(str(i) + ": " + str(obj_found))
            if (obj_found):
                count += 1
                sum += np.array([brightness, contrast, saturation])

        print("--------Change--------")
        time.sleep(5)

print(sum / count)


