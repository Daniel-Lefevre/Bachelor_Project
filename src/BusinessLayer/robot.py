from pyniryo import *
import threading
from resources.environment import configuration
import keyboard
import time
import paramiko
from queue import PriorityQueue
import copy

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
        self.conveyorSpeed = 75
        self.placeConveyor, self.placeStorage, self.observationPoseConveyor, self.observationPoseStorage = positions
        self.conveyor_id = self.robot.set_conveyor()
        self.brightnessLevelConveyor = configuration["brightness"][self.ID][0]
        self.contrastLevelConveyor = configuration["contrast"][self.ID][0]
        self.saturationLevelConveyor = configuration["saturation"][self.ID][0]
        self.brightnessLevelStorage = configuration["brightness"][self.ID][1]
        self.contrastLevelStorage = configuration["contrast"][self.ID][1]
        self.saturationLevelStorage = configuration["saturation"][self.ID][1]
        self.queue = PriorityQueue()
        self.objectUpdates = []
        self.rules = {}
        self.lock = threading.Lock()

    def startConveyorbelt(self):
        self.robot.run_conveyor(self.conveyor_id, speed=self.conveyorSpeed, direction=ConveyorDirection.BACKWARD)

    def addToQueue(self, priority, workarea, shape, color):
        self.queue.put((priority, (workarea, shape, color)))

    def setRules(self, rules):
        self.rules = rules

    def getObjectUpdates(self):
        objectUpdatesCopy = copy.deepcopy(self.objectUpdates)
        self.objectUpdates.clear()
        return objectUpdatesCopy

    def enableCamera(self):
        output = ""
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

    def inverseWorkspacepose(self, workspace_name, target_pose):
        workspace = np.array(configuration[workspace_name])
        pose = np.array([target_pose.x, target_pose.y, target_pose.z, target_pose.roll, target_pose.pitch, target_pose.yaw])
        workspace_center = np.mean(workspace, axis=0)
        difference = workspace_center - pose
        new_pose = workspace_center + difference
        new_pose[2] = target_pose.z
        return PoseObject(*new_pose)

    def graspWithTool(self):
        self.robot.grasp_with_tool()

    # Manually correct the offset of the robots (could e.g. be caused by camera distortion)
    def correctRobotOffset(self, target_pose, workspace):
        if self.ID == 1:
            target_pose = self.inverseWorkspacepose(workspace, target_pose)

        if workspace == self.StorageWorkspace:
            if (self.ID == 0):
                target_pose.z += 0.014
                target_pose.x += 0
                target_pose.y -= 0.011
            elif (self.ID == 1):
                target_pose.x += 0.006
                target_pose.y += 0
                target_pose.z += 0.013
        elif workspace == self.conveyorWorkspace:
            if (self.ID == 0):
                target_pose.x += 0.0115
                target_pose.y -= 0.0195
                target_pose.z += 0.005
            elif (self.ID == 1):
                target_pose.x += 0.008
                target_pose.y += 0.004
                target_pose.z += 0.005
        return target_pose

    def releaseWithTool(self):
        self.robot.release_with_tool()

    def findAndMoveObject(self, workspace, shape, color, destination):
        # Try to detect the object 10 times
        for i in range(10):
            obj_found, object_pose, shape_ret, color_ret = self.robot.detect_object(workspace,
                                                                                    shape=shape,
                                                                                    color=color
                                                                                    )
            if (obj_found):
                break
            print("CANT FIND OBJECT")

        if not obj_found:
            print(f"No object for robot ID: {self.ID} on {workspace}")
            self.moveToSafePosition()
            return None
        else:
            finalDestination = False
            if (destination == None):
                area = self.rules[(shape_ret, color_ret)]
                if area == "Storage":
                    destination = self.placeStorage
                    finalDestination = True
                elif area == "Conveyor":
                    destination = self.placeConveyor
                else:
                    print(f"Unknown area, {area}")

            x, y, object_yaw = object_pose

            target_pose = self.robot.get_target_pose_from_rel(workspace,
                                                                0,
                                                                x,
                                                                y,
                                                                object_yaw)
            corrected_target_pose = self.correctRobotOffset(target_pose, workspace)

            if corrected_target_pose:
                self.robot.pick_from_pose(corrected_target_pose)
                self.pickAndPlace(destination, finalDestination, shape_ret, color_ret)

    def takeObjectFromStorage(self):
        with self.lock:
            self.stopConveyorbelt()
            self.moveToObservationPositionStorage()
            self.findAndMoveObject(self.StorageWorkspace)
            self.startConveyorbelt()


    def Loop(self):
        with self.lock:
            if (self.queue.empty()):
                self.startConveyorbelt()
                all_pins = self.robot.get_digital_io_state()
                if all_pins[4].state == PinState.LOW:
                    self.stopConveyorbelt()
                    time.sleep(0.5)
                    self.addToQueue(configuration["PickFromIRSensorPriority"], "Conveyor", ObjectShape.ANY, ObjectColor.ANY)
            else:
                self.stopConveyorbelt()
                _, (workarea, shape, color) = self.queue.get()
                self.objectUpdates.append((shape, color, "In_Transit"))

                if (workarea=="Conveyor"):
                    self.moveToObservationPositionConveyor()
                    destination = None
                    self.findAndMoveObject(self.conveyorWorkspace, shape, color, destination)
                elif (workarea=="Storage"):
                    self.moveToObservationPositionStorage()
                    destination = self.placeConveyor
                    self.findAndMoveObject(self.StorageWorkspace, shape, color, destination)
                else:
                    print(f"Workarea not defined. Workearea was {workarea}, but must be 'Storage' og 'Conveyor'")

    def setCameraSettings(self, workarea):
        print(f"Changing Camera Settings for {self.ID}")
        if (workarea == "Conveyor"):
            self.robot.set_brightness(self.brightnessLevelConveyor)
            self.robot.set_contrast(self.contrastLevelConveyor)
            self.robot.set_saturation(self.saturationLevelConveyor)
        elif (workarea == "Storage"):
            self.robot.set_brightness(self.brightnessLevelStorage)
            self.robot.set_contrast(self.contrastLevelStorage)
            self.robot.set_saturation(self.saturationLevelStorage)
        else:
            print(f"Workarea not defined. Workearea was {workarea}, but must be 'Storage' og 'Conveyor'")


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
        self.setCameraSettings("Conveyor")

    def moveToObservationPositionStorage(self):
        self.robot.move_pose(*self.observationPoseStorage)
        self.setCameraSettings("Storage")

    def placeAndRelease(self, destination):
        self.robot.move_pose(*destination)
        self.releaseWithTool()

    def pickAndPlace(self, destination, finalDestination, shape, color):
        self.moveToSafePosition()
        self.placeAndRelease(destination)
        if (finalDestination):
            self.objectUpdates.append((shape, color, f"Storage_{self.ID}"))

        self.moveToSafePosition()
        self.moveToObservationPositionConveyor()


    def disconnect(self):
        self.stopConveyorbelt()
        self.robot.unset_conveyor(self.conveyor_id)
        self.robot.close_connection()
        print(f"Connection closed, ID: {self.ID}")
