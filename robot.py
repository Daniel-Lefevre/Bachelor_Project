from pyniryo import *
import threading
from environment import configuration
import keyboard
import time
import paramiko

class RobotArm:
    def __init__(self, ip, positions, ID):
        self.ip = ip
        self.robot = NiryoRobot(ip)
        self.ID = ID
        self.cameraTopic = "/niryo_robot_vision/compressed_video_stream"
        self.setupFile = "/home/niryo/catkin_ws/install/release/ned2/setup.bash"
        self.conveyorWorkspace = f"Conveyor_workspace_{self.ID}"
        self.StorageWorkspace = f"Storage_workspace_{self.ID}"
        self.safePosition = [0.0, 0.0, 0.0, 0.0, -1.57, 0.0]
        self.conveyorSpeed = 50
        self.placeConveyor, self.placeStorage, self.observationPoseConveyor, self.observationPoseStorage = positions
        self.conveyor_id = self.robot.set_conveyor()
        self.brightnessLevel = configuration["brightness"][self.ID]
        self.contrastLevel = configuration["contrast"][self.ID]
        self.saturationLevel = configuration["saturation"][self.ID]
        self.lock = threading.Lock()

    def startConveyorbelt(self):
        self.robot.run_conveyor(self.conveyor_id, speed=self.conveyorSpeed, direction=ConveyorDirection.BACKWARD)

    def enableCamera(self):
        output = ""
        print(f"Trying to enable camera on robot {self.ID}")
        # Connect to the robot via SSH
        with paramiko.SSHClient() as client:
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                client.connect(self.ip, username="niryo", password="robotics", timeout=3)
                full_cmd = (
                    f"source {self.setupFile} && "   # Source the setup file
                    f"rosservice call /niryo_robot_vision/start_stop_video_streaming '{{value: true}}' && " # Enable camera
                    f"timeout 2s rostopic hz {self.cameraTopic}" # Run the ros topic for 2 seconds to see if the camera publishes anything (saving the output in the next commando)
                )
                _, stdout, _ = client.exec_command(full_cmd)
                # Wait for the command to finish (because we have a 2 second timer in the last command)
                stdout.channel.recv_exit_status()
                output = stdout.read().decode()
            except Exception as e:
                print(f"SSH to check camera failed: {e}")
                return False

        return ("average rate" in output)

    def takeImage(self):
        with self.lock:
            # Take in image
            img_compressed = self.robot.get_img_compressed()
            if (img_compressed):
                # Uncompress and save the image
                img = uncompress_image(img_compressed)
                cv2.imwrite("final_result.jpg", img)
                print("Saved image")
            else:
                print("Could not take image")

    def stopConveyorbelt(self):
        self.robot.stop_conveyor(self.conveyor_id)

    def moveToSafePosition(self):
        self.robot.move_joints(self.safePosition)

    def graspWithTool(self):
        self.robot.grasp_with_tool()

    def releaseWithTool(self):
        self.robot.release_with_tool()

    def findObjectInWorkspace(self, workspace):
        # Try to detect the object 10 times
        for i in range(10):
            obj_found, object_pose, shape_ret, color_ret = self.robot.detect_object(workspace,
                                                                                    shape=ObjectShape.ANY,
                                                                                    color=ObjectColor.ANY
                                                                                    )
            if (obj_found):
                break

        if not obj_found:
            print(f"No object for robot ID: {self.ID} on {workspace}")
            self.moveToSafePosition()
            self.moveToObservationPositionConveyor()
            return None
        else:
            print(f"Object found for robot ID: {self.ID} on {workspace}")
            print(f"Object: {shape_ret}, {color_ret}")
            x, y, object_yaw = object_pose

            target_pose = self.robot.get_target_pose_from_rel(workspace,
                                                                0,
                                                                x,
                                                                y,
                                                                object_yaw)
            # Manually correct the offset of the robots (could e.g. be caused by camera distortion)
            if workspace == self.StorageWorkspace:
                if (self.ID == 0):
                    target_pose.z += 0.014
                    target_pose.x += 0
                    target_pose.y -= 0.011
                elif (self.ID == 1):
                    target_pose.x -= 0.02
                    target_pose.y += 0.01
                    target_pose.z += 0.014
            elif workspace == self.conveyorWorkspace:
                if (self.ID == 0):
                    target_pose.x += 0.005
                    target_pose.y -= 0.013
                    target_pose.z += 0.007
                elif (self.ID == 1):
                    target_pose.x += 0.02
                    target_pose.y += 0.01
                    target_pose.z += 0.007

            return target_pose

    def takeObjectFromStorage(self):
        with self.lock:
            self.stopConveyorbelt()
            self.moveToObservationPositionStorage()
            print("Moved to observation storage")
            target_pose = self.findObjectInWorkspace(self.StorageWorkspace)
            if target_pose:
                self.robot.pick_from_pose(target_pose)
                self.pickAndPlace()
            self.startConveyorbelt()


    def Loop(self):
        with self.lock:
            all_pins = self.robot.get_digital_io_state()
            if all_pins[4].state == PinState.LOW:
                self.stopConveyorbelt()
                time.sleep(0.5)
                target_pose = self.findObjectInWorkspace(self.conveyorWorkspace)

                self.robot.pick_from_pose(target_pose)
                self.pickAndPlace()


    def setUp(self):
        # Check if there is a connection to the robot
        if (self.robot.hardware_status.connection_up):
            print(f"Connection to robot: {self.ID} successful")

        # Enable the camera on the robot
        if(not self.enableCamera()):
            print("Camera could not get enabled")
            self.disconnect()
            return
        print(f"Camera on robot {self.ID} has been enabled")

        # Camera settings
        self.robot.set_brightness(self.brightnessLevel)
        self.robot.set_contrast(self.contrastLevel)
        self.robot.set_saturation(self.saturationLevel)

        # Start the conveyor belt
        self.startConveyorbelt()

        # Update the tool of the robot (make it autodetect)
        self.robot.update_tool()

        # Release the vacuum
        self.robot.release_with_tool()
        print(f"Done with setup ID: {self.ID}")

        # Calibrate the robot if needed
        if (self.robot.hardware_status.calibration_needed):
            print("Calibrating...")
            self.robot.calibrate_auto()

        # Move to safe position
        print("Moving to safe position")
        self.moveToSafePosition()
        self.moveToObservationPositionConveyor()

        # Save the workspace defined in environment
        if (self.conveyorWorkspace not in self.robot.get_workspace_list()):
            self.robot.save_workspace_from_robot_poses(self.conveyorWorkspace, *configuration[self.conveyorWorkspace])

        if (self.StorageWorkspace not in self.robot.get_workspace_list()):
            self.robot.save_workspace_from_robot_poses(self.StorageWorkspace, *configuration[self.StorageWorkspace])

    def moveToObservationPositionConveyor(self):
        self.robot.move_pose(*self.observationPoseConveyor)

    def moveToObservationPositionStorage(self):
        self.robot.move_pose(*self.observationPoseStorage)

    def place(self):
        self.robot.move_pose(*self.placeConveyor)
        self.releaseWithTool()

    def pickAndPlace(self):
        print("move to safe")
        self.moveToSafePosition()
        print("safe pos")
        self.place()
        self.moveToSafePosition()
        self.moveToObservationPositionConveyor()
        self.startConveyorbelt()
        print(f"Pick up and place, ID: {self.ID}")

    def disconnect(self):
        self.stopConveyorbelt()
        self.robot.unset_conveyor(self.conveyor_id)
        self.robot.close_connection()
        print(f"Connection closed, ID: {self.ID}")


class System:
    def __init__(self, ips, positions):
        self.RobotArms = []
        self.running = True

        # Add all the robot arms
        for i in range(len(ips)):
            ID = i
            IP_address = ips[i]
            poses = positions[i]
            self.RobotArms.append(RobotArm(IP_address, poses, ID))

    def robot_worker(self, arm):
        # Setup Phase
        print(f"Robot {arm.ID} is initializing")
        arm.setUp()
        print(f"Robot {arm.ID} setup finished")

        # 2. Monitoring Phase
        while self.running:
            arm.Loop()
            time.sleep(0.1)

    def listenToIR(self):
        self.threads = []

        # Start one thread per robot arm
        for arm in self.RobotArms:
            t = threading.Thread(target=self.robot_worker, args=(arm,))
            t.daemon = True
            self.threads.append(t)
            t.start()

        # Check for events to do
        while self.running:
            # Robot 0 grab object from storage
            if keyboard.is_pressed('0'):
                self.RobotArms[0].takeObjectFromStorage()
                time.sleep(0.5)

            # Robot 1 grab object from storage
            elif keyboard.is_pressed('1'):
                self.RobotArms[1].takeObjectFromStorage()
                time.sleep(0.5)

            # Robot 0 take image
            elif keyboard.is_pressed('9'):
                self.RobotArms[0].takeImage()
                time.sleep(0.5)

            # Robot 1 take image
            elif keyboard.is_pressed('2'):
                self.RobotArms[1].takeImage()
                time.sleep(0.5)

            # Shutdown event
            elif keyboard.is_pressed('s'):
                print("Shutting Down")
                self.running = False
            else:
                time.sleep(0.1)

        # Wait for the threads to finnish their task before shutting down
        for t in self.threads:
            t.join()

        # Disconnect the arms
        for arm in self.RobotArms:
            arm.disconnect()

        print("Everything has been shut down")


if __name__ == "__main__":
    IPs = configuration["ips"]
    positions = configuration["positions"]
    system = System(IPs, positions)
    system.listenToIR()
