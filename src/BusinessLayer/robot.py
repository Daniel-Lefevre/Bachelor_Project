import copy
import threading
import time

import cv2
import numpy as np
import paramiko
from pyniryo import ConveyorDirection, NiryoRobot, ObjectColor, ObjectShape, PinState, PoseObject, uncompress_image

from resources.environment import configuration
from resources.PriorityQueue import CustomPriorityQueue


class RobotArm:
    def __init__(self, ip: str, positions: list[float], id: int, initial_object_positions: list):
        self.ip = ip
        self.robot = NiryoRobot(ip)
        self.ID = id
        self.camera_topic = "/niryo_robot_vision/compressed_video_stream"
        self.setup_file = "/home/niryo/catkin_ws/install/release/ned2/setup.bash"
        self.conveyor_workspace = f"Conveyor_workspace_{self.ID}"
        self.storage_workspace = f"Storage_workspace_{self.ID}"
        self.conveyor_speed = 75
        self.place_conveyor, self.observation_pose, self.observation_pose_storage, self.standby_position, self.observation_pose_conveyor = positions
        self.place_storage = configuration["storagePositions"][self.ID]
        self.occupied_storage = initial_object_positions
        self.conveyor_id = self.robot.set_conveyor()
        self.brightness_level_conveyor = configuration["brightness"][self.ID][0]
        self.contrast_level_conveyor = configuration["contrast"][self.ID][0]
        self.saturation_level_conveyor = configuration["saturation"][self.ID][0]
        self.brightness_level_storage = configuration["brightness"][self.ID][1]
        self.contrast_level_storage = configuration["contrast"][self.ID][1]
        self.saturation_level_storage = configuration["saturation"][self.ID][1]
        self.queue = CustomPriorityQueue(configuration["NumberOfPriorities"])
        self.object_updates = []
        self.anomaly_updates = []
        self.latest_image = None
        self.IR = False
        self.rules = {}
        self.lock = threading.Lock()
        self.mitigation_mode = False
        self.pick_and_place_first_try = True
        self.ready_to_drop = False

    def get_ir(self) -> bool:
        return self.IR

    def _start_conveyorbelt(self) -> None:
        self.robot.run_conveyor(self.conveyor_id, speed=self.conveyor_speed, direction=ConveyorDirection.BACKWARD)
        # self.robot.run_conveyor(self.conveyor_id, speed=0, direction=ConveyorDirection.BACKWARD)

    def set_mitigation_mode(self, value) -> None:
        self.mitigation_mode = value

    def get_rules(self) -> dict:
        return self.rules

    def add_to_queue(self, priority: int, workarea: str, shape: ObjectShape, color: ObjectColor) -> None:
        self.queue.put((priority, (workarea, shape, color)))

    def set_rules(self, rules: dict) -> None:
        for rule_key in rules:
            self.rules[rule_key] = rules[rule_key]

    def remove_object_from_storage(self, shape: ObjectShape, color: ObjectColor) -> None:
        print(f"storage: {self.occupied_storage}")
        for i in range(len(self.occupied_storage)):
            if self.occupied_storage[i] is not None:
                if self.occupied_storage[i][0] == shape and self.occupied_storage[i][1] == color:
                    self.occupied_storage[i] = None
                    print(self.occupied_storage)
                    return

    def get_object_updates(self) -> list[tuple[ObjectShape, ObjectColor, str]]:
        object_updates_copy = copy.deepcopy(self.object_updates)
        self.object_updates.clear()
        return object_updates_copy

    def get_anomaly_updates(self) -> list[tuple[str, tuple[int, ObjectShape, ObjectColor] | None]]:
        anomaly_updates_copy = copy.deepcopy(self.anomaly_updates)
        self.anomaly_updates.clear()
        return anomaly_updates_copy

    def drop_object(self) -> None:
        self.ready_to_drop = True

    def get_latest_image(self) -> np.ndarray | None:
        image = copy.deepcopy(self.latest_image)
        self.latest_image = None
        return image

    def _enable_camera(self) -> bool:
        output = ""
        # Connect to the robot via SSH
        with paramiko.SSHClient() as client:
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                client.connect(self.ip, username="niryo", password="robotics", timeout=3)
                full_cmd = (
                    f"source {self.setup_file} && "  # Source the setup file
                    f"rosservice call /niryo_robot_vision/start_stop_video_streaming '{{value: true}}' && "  # Enable camera
                    f"timeout 2s rostopic hz {self.camera_topic}"  # Run the ros topic for 2 seconds to see if the camera publishes anything (saving the output in the next commando)
                )
                _, stdout, _ = client.exec_command(full_cmd)
                # Wait for the command to finish (because we have a 2 second timer in the last command)
                stdout.channel.recv_exit_status()
                output = stdout.read().decode()
            except Exception as e:
                print(f"SSH to check camera failed: {e}")
                return False

        return "average rate" in output

    def _take_image(self) -> np.ndarray:
        with self.lock:
            img_compressed = self.robot.get_img_compressed()
            img_bgr = uncompress_image(img_compressed)
            image_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        return image_rgb

    def _stop_conveyorbelt(self) -> None:
        self.robot.stop_conveyor(self.conveyor_id)

    def _move_to_standby_position(self) -> None:
        self.robot.move_pose(self.standby_position)

    def _inverse_workspacepose(self, workspace_name: str, target_pose: PoseObject) -> PoseObject:
        workspace = np.array(configuration[workspace_name])
        pose = np.array([target_pose.x, target_pose.y, target_pose.z, target_pose.roll, target_pose.pitch, target_pose.yaw])
        workspace_center = np.mean(workspace, axis=0)
        difference = workspace_center - pose
        new_pose = workspace_center + difference
        new_pose[2] = target_pose.z
        new_pose[3] = target_pose.roll
        new_pose[4] = target_pose.pitch
        new_pose[5] = target_pose.yaw
        return PoseObject(*new_pose)

    #
    def _grasp_with_tool(self) -> None:
        self.robot._grasp_with_tool()

    # Manually correct the offset of the robots (could e.g. be caused by camera distortion)
    def _correct_robot_offset(self, target_pose: PoseObject, workspace: str) -> PoseObject:
        if self.ID == 1:
            target_pose = self._inverse_workspacepose(workspace, target_pose)

        if workspace == self.storage_workspace:
            if self.ID == 0:
                target_pose.z += 0.016
                target_pose.x += 0
                target_pose.y -= 0.011
            elif self.ID == 1:
                target_pose.x += 0.01
                target_pose.y += 0
                target_pose.z -= 0.007
        elif workspace == self.conveyor_workspace:
            if self.ID == 0:
                target_pose.x += 0.0115
                target_pose.y -= 0.0095
                target_pose.z += 0.005
            elif self.ID == 1:
                target_pose.x += 0.018
                target_pose.y += 0.018
                target_pose.z += 0.005
        return target_pose

    def _release_with_tool(self) -> None:
        self.robot.release_with_tool()

    def _find_and_move_object(self, workspace: str, shape: ObjectShape, color: ObjectColor, destination: list[float] | None) -> None:
        # Try to detect the object 10 times
        for _ in range(10):
            obj_found, object_pose, shape_ret, color_ret = self.robot.detect_object(workspace, shape=shape, color=color)
            if obj_found:
                break
            print("CANT FIND OBJECT")

        if not obj_found:
            if destination is not None:  # Object is taken from storage
                self.anomaly_updates.append(("Anomaly 14",))

        else:
            # print(f"++++++++++Object found: {shape_ret}, {color_ret}++++++++++++")
            final_destination = False
            if destination is None:
                area = self.rules.get((shape_ret, color_ret))
                if area is None:
                    # The wrong object is on the conveyor belt, cast anomaly 5
                    self.set_mitigation_mode(True)
                    self.anomaly_updates.append(("Anomaly 5", (self.ID, shape_ret, color_ret)))
                    return

                if area == "Storage":
                    for i in range(len(self.occupied_storage)):
                        if self.occupied_storage[i] is None:
                            destination = self.place_storage[i]
                            print(i, destination)
                            final_destination = True
                            self.occupied_storage[i] = (shape_ret, color_ret)
                            break
                        elif i == len(self.occupied_storage) - 1:
                            self.set_mitigation_mode(True)
                            self.anomaly_updates.append(("Anomaly 11", (self.ID, shape_ret, color_ret)))
                            return
                elif area == "Conveyor":
                    destination = self.place_conveyor
                else:
                    print(f"Unknown area, {area}")
                    return
            else:
                self.remove_object_from_storage(shape, color)

            x, y, object_yaw = object_pose

            target_pose = self.robot.get_target_pose_from_rel(workspace, 0, x, y, object_yaw)
            corrected_target_pose = self._correct_robot_offset(target_pose, workspace)

            if corrected_target_pose:
                self.robot.pick_from_pose(corrected_target_pose)
                self._pick_and_place(destination, final_destination, shape_ret, color_ret, workspace)

    def _check_ir(self) -> bool:
        all_pins = self.robot.get_digital_io_state()
        self.IR = all_pins[4].state == PinState.LOW
        return self.IR

    def loop(self) -> None:
        if self.mitigation_mode:
            return
        with self.lock:
            if self.queue.empty():
                if not self._check_ir():
                    self._start_conveyorbelt()
                else:
                    self.add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", ObjectShape.ANY, ObjectColor.ANY)
            else:
                self._stop_conveyorbelt()
                time.sleep(0.5)
                workarea, shape, color = self.queue.get()
                self.object_updates.append((shape, color, "In_Transit"))

                if workarea == "Conveyor":
                    self._move_to_observation_position_conveyor()
                    destination = None
                    self._find_and_move_object(self.conveyor_workspace, shape, color, destination)
                elif workarea == "Storage":
                    self._move_to_observation_position_storage()
                    destination = self.place_conveyor
                    self._find_and_move_object(self.storage_workspace, shape, color, destination)
                else:
                    print(f"Workarea not defined. Workearea was {workarea}, but must be 'Storage' og 'Conveyor'")

    def _set_camera_settings(self, workarea: str) -> None:
        print(f"Changing Camera Settings for {self.ID}")
        if workarea == "Conveyor":
            self.robot.set_brightness(self.brightness_level_conveyor)
            self.robot.set_contrast(self.contrast_level_conveyor)
            self.robot.set_saturation(self.saturation_level_conveyor)
        elif workarea == "Storage":
            self.robot.set_brightness(self.brightness_level_storage)
            self.robot.set_contrast(self.contrast_level_storage)
            self.robot.set_saturation(self.saturation_level_storage)
        else:
            print(f"Workarea not defined. Workearea was {workarea}, but must be 'Storage' og 'Conveyor'")

    def set_up(self) -> None:
        # Check if there is a connection to the robot
        if self.robot.hardware_status.connection_up:
            print(f"Connection to robot: {self.ID} successful")

        # Enable the camera on the robot
        if not self._enable_camera():
            print("Camera could not get enabled")
            self.disconnect()
            return
        print(f"Camera on robot {self.ID} has been enabled")

        # Start the conveyor belt
        self._start_conveyorbelt()

        # Update the tool of the robot (make it autodetect)
        self.robot.update_tool()

        # Release the vacuum
        self._release_with_tool()
        print(f"Done with setup ID: {self.ID}")

        # Calibrate the robot if needed
        if self.robot.hardware_status.calibration_needed:
            print("Calibrating...")
            self.robot.calibrate_auto()

        # Move to safe position
        print("Moving to safe position")
        self._move_to_observation_position()

        # Save the workspace defined in environment
        if self.conveyor_workspace not in self.robot.get_workspace_list():
            self.robot.save_workspace_from_robot_poses(self.conveyor_workspace, *configuration[self.conveyor_workspace])

        if self.storage_workspace not in self.robot.get_workspace_list():
            self.robot.save_workspace_from_robot_poses(self.storage_workspace, *configuration[self.storage_workspace])

    def _move_to_observation_position(self) -> None:
        self.robot.move_pose(*self.observation_pose)
        self.latest_image = self._take_image()

    def _move_to_observation_position_storage(self) -> None:
        self.robot.move_pose(*self.observation_pose_storage)
        self._set_camera_settings("Storage")

    def _move_to_observation_position_conveyor(self) -> None:
        self.robot.move_pose(*self.observation_pose_conveyor)
        self._set_camera_settings("Conveyor")

    def _place_and_release(self, destination: list[float]) -> None:
        if destination == self.place_conveyor:
            self._move_to_standby_position()
            start_time = time.time()

            while not self.ready_to_drop:
                if self._check_ir():
                    self._stop_conveyorbelt()
                elif time.time() - start_time > 1:
                    self._start_conveyorbelt()

                time.sleep(0.01)

            self._stop_conveyorbelt()
            self.ready_to_drop = False

        self.robot.move_pose(*destination)
        self._release_with_tool()

    def _pick_and_place(self, destination: list[float], final_destination: bool, shape: ObjectShape, color: ObjectColor, workspace: str) -> None:
        if workspace == self.conveyor_workspace:
            self._move_to_observation_position_conveyor()

        if self._check_ir():
            if self.pick_and_place_first_try:
                self.pick_and_place_first_try = False
                # Robot arm has failed to pickup object from the conveyor, cast anomaly 4
                self.anomaly_updates.append(("Anomaly 4", (self.ID, shape, color)))
                self.remove_object_from_storage(shape, color)
                self._release_with_tool()
                self._find_and_move_object(workspace, shape, color, None)
                return
            else:
                self.remove_object_from_storage(shape, color)
                self.anomaly_updates.append(("Anomaly 4 Mitigation failed",))
                return

        self._move_to_observation_position()

        if workspace == self.conveyor_workspace:
            self.rules.pop((shape, color), None)

        self.pick_and_place_first_try = True
        self._place_and_release(destination)
        if final_destination:
            self.object_updates.append((shape, color, f"Storage_{self.ID}"))

        self._move_to_observation_position()

    def disconnect(self) -> None:
        self._stop_conveyorbelt()
        self.robot.unset_conveyor(self.conveyor_id)
        self.robot.close_connection()
        print(f"Connection closed, ID: {self.ID}")
