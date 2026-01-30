from pyniryo import *
from environment import configuation
import keyboard
import time
import paramiko

class RobotArm:
    def __init__(self, ip, positions, ID):
        self.robot = NiryoRobot(ip)
        self.ID = ID
        self.conveyorWorkspace = f"Conveyor_workspace_{self.ID}"
        self.StorageWorkspace = f"Storage_workspace_{self.ID}"
        self.safePosition = [0.0, 0.0, 0.0, 0.0, -1.57, 0.0]
        self.conveyorSpeed = 50
        self.placeConveyor, self.placeStorage, self.observationPoseConveyor, self.observationPoseStorage = positions
        self.conveyor_id = self.robot.set_conveyor()
        self.brightnessLevel = configuation["brightness"][self.ID]
        self.contrastLevel = configuation["contrast"][self.ID]
        self.saturationLevel = configuation["saturation"][self.ID]

    def startConveyorbelt(self):
        self.robot.run_conveyor(self.conveyor_id, speed=self.conveyorSpeed, direction=ConveyorDirection.BACKWARD)

    def stopConveyorbelt(self):
        self.robot.stop_conveyor(self.conveyor_id)

    def moveToSafePosition(self):
        self.robot.move_joints(self.safePosition)

    def graspWithTool(self):
        self.robot.grasp_with_tool()
    
    def releaseWithTool(self):
        self.robot.release_with_tool()

    def checkIR(self):
        all_pins = self.robot.get_digital_io_state()
        if all_pins[4].state == PinState.LOW:
            self.stopConveyorbelt()

            obj_found, object_pose, shape_ret, color_ret = self.robot.detect_object(self.conveyorWorkspace,
                                                                                    shape=ObjectShape.ANY,
                                                                                    color=ObjectColor.ANY
                                                                                    )
            if not obj_found:
                print(f"No object for robot ID: {self.ID}")
            else:
                print(f"Object found for robot ID: {self.ID}")
                print(f"Object: {shape_ret}, {color_ret}")
                print(object_pose)
                x, y, object_yaw = object_pose
                target_pose = self.robot.get_target_pose_from_rel(self.conveyorWorkspace,
                                                                    0,
                                                                    x,
                                                                    y,
                                                                    object_yaw)

                self.robot.pick_from_pose(target_pose)
                self.pickAndPlace()

        
    def setUp(self):
        # Start the conveyor belt
        self.startConveyorbelt()

        # Camera settings
        self.robot.set_brightness(self.brightnessLevel)
        self.robot.set_contrast(self.contrastLevel)
        self.robot.set_saturation(self.saturationLevel)

        # Update the tool of the robot (make it autodetect)
        self.robot.update_tool()
        
        # Calibrate the robot
        print("Calibrating...")
        self.robot.calibrate_auto()

        # Move to safe position
        print("Moving to safe position")
        self.moveToSafePosition()
        self.moveToObservationPosition()

        # Release the vacuum
        self.robot.release_with_tool()
        print(f"Done with setup ID: {self.ID}")

        # Save the workspace defined in environment
        if (self.conveyorWorkspace not in self.robot.get_workspace_list()):
            self.robot.save_workspace_from_robot_poses(self.conveyorWorkspace, *configuation[self.conveyorWorkspace])

        if (self.StorageWorkspace not in self.robot.get_workspace_list()):
            self.robot.save_workspace_from_robot_poses(self.StorageWorkspace, *configuation[self.StorageWorkspace])

    def moveToObservationPosition(self):
        self.robot.move_pose(self.observationPoseConveyor)        
    
    def place(self, storage):
        if storage:
            self.robot.move_pose(*self.observationPoseConveyor)
        else:
            self.robot.move_pose(*self.placeConveyor)
        self.releaseWithTool()

        
    def pickAndPlace(self):
        self.moveToSafePosition()
        self.place(False)
        self.startConveyorbelt()
        self.moveToSafePosition()
        self.moveToObservationPosition()
        print(f"Pick up and place, ID: {self.ID}")
        
    def disconnect(self):
        self.stopConveyorbelt()
        self.robot.unset_conveyor(self.conveyor_id)
        self.robot.close_connection()
        print(f"Connection closed, ID: {self.ID}")


class System:
    def __init__(self, ips, positions):
        self.RobotArms = []
        # Add all the robot arms
        for i in range(len(ips)):
            ID = i
            IP_address = ips[i]
            poses = positions[i]
            self.RobotArms.append(RobotArm(IP_address, poses, ID))

    def setUp(self):
        # Setup the robotarms and conveyor belts
        for RobotArm in self.RobotArms:
            RobotArm.setUp()
            
    def listenToIR(self):
        while(True):
            for RobotArm in self.RobotArms:
                RobotArm.checkIR()
                
            if keyboard.is_pressed('s'):
                print("Manual stop triggered by user.")
                for RobotArm in self.RobotArms:
                    RobotArm.disconnect()
                break
            
            #time.sleep(0.1)



# --- CONFIG ---
ROBOT_IP = "169.254.200.200"
USER = "niryo"
PASS = "robotics"
TOPIC = "/niryo_robot_vision/compressed_video_stream"
# --------------
def check_status():
    print(f"Connecting to {ROBOT_IP}...")
    
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(ROBOT_IP, username=USER, password=PASS, timeout=5)

        # We run 'rostopic hz' for 2 seconds. 
        # If the camera is sending data, this command prints the rate (e.g. "average rate: 10.5").
        # If the camera is OFF, it prints nothing and times out.
        cmd = f"timeout 2s rostopic hz {TOPIC}"
        
        stdin, stdout, stderr = client.exec_command(cmd)
        output = stdout.read().decode()
        
        # Check the result
        if "average rate" in output:
            print("\n✅ RESULT: CAMERA IS ON")
            print("(Data is flowing correctly)")
        else:
            print("\n❌ RESULT: CAMERA IS OFF")
            print("(No data detected on the video topic)")

        client.close()

    except Exception as e:
        print(f"❌ Connection Failed: {e}")

if __name__ == "__main__":
    check_status()
    IPs = configuation["ips"]
    positions = configuation["positions"]
    system = System(IPs, positions)
    
    system.setUp()
    system.listenToIR()

