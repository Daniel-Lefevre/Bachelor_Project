from __future__ import annotations

import copy
import threading
import time
from typing import TYPE_CHECKING

from resources.environment import StorageObject, configuration
from src.BusinessLayer.DT.dt_runner import DTRunner
from src.BusinessLayer.robot import RobotArm

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape


class System:
    def __init__(self, ips: list[str], positions: list[list[float]]):
        self.robot_arms = []
        self.running = True
        self.storage_objects = configuration["StorageObjects"]
        self.DT = DTRunner()
        self.lock = threading.Lock()
        self.DT.create_event(("Setup done", None))

        # Add all the robot arms
        for i in range(len(ips)):
            id_value = i
            ip_address = ips[i]
            poses = positions[i]
            self.robot_arms.append(RobotArm(ip_address, poses, id_value, copy.deepcopy(configuration["StorageOccupancy"][id_value])))

    #
    #
    # Private functions
    #
    #

    # Sets up the connections to the robot arms
    def _robot_worker(self, arm: RobotArm) -> None:
        # set up Phase
        print(f"Robot {arm.ID} is initializing")
        arm.set_up()
        print(f"Robot {arm.ID} setup finished")

        # Change DT robots state to next state after setup
        self.DT.create_event(("Setup done", None))

        # 2. Monitoring Phase
        while self.running:
            arm.loop()
            time.sleep(0.05)

        # Disconnect when not running
        arm.disconnect()
        print("Everything has been shut down")

    # Updates storage object to its new position
    def _update_object(self, shape: ObjectShape, color: ObjectColor, position: str) -> None:
        with self.lock:
            for i in range(len(self.storage_objects)):
                obj = self.storage_objects[i]
                if obj.shape == shape and obj.color == color:
                    self.storage_objects[i].position = position
                    return

    # Helper function for finding storage object by name
    def _find_object_by_name(self, object_name: str) -> StorageObject:
        for object in self.storage_objects:
            if object.name == object_name:
                return object

    # Make the rules for the robots based on event
    def _make_rule_from_event(self, object: StorageObject, destination: str) -> list[dict]:
        rules = [{}, {}]
        # Move to different storage
        storage_id = int(object.position[-1])
        opposite_id = int(not storage_id)
        if object.position != destination:
            rules[opposite_id][(object.shape, object.color)] = "Storage"

        # Move to same storage
        else:
            rules[opposite_id][(object.shape, object.color)] = "Conveyor"
            rules[storage_id][(object.shape, object.color)] = "Storage"

        return rules

    # Listen for anomalies detected by the robotarms
    # And send the anomalies to the digital twin
    def _anomaly_listener(self) -> None:
        while self.running:
            for robot_id, robot_arm in enumerate(self.robot_arms):
                messages = robot_arm.get_anomaly_updates()
                for message in messages:
                    if message[0] in ["Anomaly 4 Mitigation failed", "Anomaly 14"]:
                        self._create_dt_anomaly_event(message[0], robot_id)
                        self.stop_system()
                    elif message[0] == "Anomaly 4":
                        id_value, shape, color = message[1]
                        self._create_dt_anomaly_event(message[0], id_value, shape, color)
                    elif message[0] == "Anomaly 5":
                        id_value, shape, color = message[1]
                        self._create_dt_anomaly_event(message[0], id_value, shape, color)
                        self._anomaly_5_mitigation(id_value, shape, color)
                    elif message[0] == "Anomaly 11":
                        id_value, shape, color = message[1]
                        self._create_dt_anomaly_event(message[0], id_value, shape, color)
                        self._anomaly_11_mitigation(id_value, shape, color)

            time.sleep(0.1)

    # Helper function to create the anomaly event in the DT
    def _create_dt_anomaly_event(self, eventype: str, robot_id: int = None, shape: ObjectShape = None, color: ObjectColor = None) -> None:
        event_param = (robot_id, shape, color)
        self.DT.create_event((eventype, event_param))

    # Create rules for the robotarms to mitigat anomaly 11
    def _anomaly_11_mitigation(self, robot_id_arrival: int, shape: ObjectShape, color: ObjectColor) -> None:
        # Rules for physical system
        self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})
        self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})
        self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
        self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

        # Rules for Digital twin
        self.DT.set_rules([None, {(shape, color): "Conveyor"}]) if robot_id_arrival else self.DT.set_rules([{(shape, color): "Conveyor"}, None])
        self.DT.set_rules([{(shape, color): "Storage"}, None]) if robot_id_arrival else self.DT.set_rules([None, {(shape, color): "Storage"}])

    # Create rules for the robotarms to mitigat anomaly 5
    def _anomaly_5_mitigation(self, robot_id_arrival: int, shape: ObjectShape, color: ObjectColor) -> None:
        goal_storage_id = None
        for storage_object in self.storage_objects:
            if storage_object.shape == shape and storage_object.color == color:
                if storage_object.position == "In_Transit":
                    for robot_id in range(len(self.robot_arms)):
                        if self.robot_arms[robot_id].get_rules().get((shape, color)) == "Storage":
                            goal_storage_id = robot_id
                else:
                    goal_storage_id = int(storage_object.position[-1])
                    self.robot_arms[goal_storage_id].remove_object_from_storage(shape, color)
                break
        if goal_storage_id == robot_id_arrival:
            self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Storage"})
            self.DT.set_rules([None, {(shape, color): "Storage"}]) if robot_id_arrival else self.DT.set_rules([{(shape, color): "Storage"}, None])

            self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
            self.robot_arms[robot_id_arrival].set_mitigation_mode(False)
        else:
            self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})
            self.DT.set_rules([None, {(shape, color): "Conveyor"}]) if robot_id_arrival else self.DT.set_rules([{(shape, color): "Conveyor"}, None])

            self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})
            self.DT.set_rules([{(shape, color): "Storage"}, None]) if robot_id_arrival else self.DT.set_rules([None, {(shape, color): "Storage"}])

            self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
            self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

    # Listens to the IR sensor, if the IR just switched from false to true, then create an event in the DT
    def _ir_listener(self) -> None:
        has_received_false = [False, False]
        while self.running:
            for id in range(len(self.robot_arms)):
                robot_arm = self.robot_arms[id]
                if robot_arm.get_ir():
                    if has_received_false[id]:
                        with self.lock:
                            self.DT.create_event(("IR", id))
                        has_received_false[id] = False
                else:
                    has_received_false[id] = True

            time.sleep(0.1)

    def _image_listener(self) -> None:
        while self.running:
            for robot_id in range(len(self.robot_arms)):
                image = self.robot_arms[robot_id].get_latest_image()
                if image is not None:
                    with self.lock:
                        event_param = (image, robot_id)
                        self.DT.create_event(("Image", event_param))

            time.sleep(0.1)

    #
    #
    # Public functions
    #
    #

    # Stop the system from running, which means we cannot control the system anymore
    def stop_system(self) -> None:
        print("STOP")
        self.running = False
        self.DT.stop_dt()

        # Wait for the threads to finnish their task before shutting down
        current_thread = threading.current_thread()
        if hasattr(self, "threads"):
            for t in self.threads:
                if t is not current_thread:
                    t.join()

    # Setup all the listeners in system
    def set_up(self) -> None:
        self.DT.start_dt()

        self.threads = []

        t_ir = threading.Thread(target=self._ir_listener)
        self.threads.append(t_ir)
        t_ir.start()

        for arm in self.robot_arms:
            t = threading.Thread(target=self._robot_worker, args=(arm,))
            self.threads.append(t)
            t.start()

        t_anomaly = threading.Thread(target=self._anomaly_listener)
        self.threads.append(t_anomaly)
        t_anomaly.start()

        t_image = threading.Thread(target=self._image_listener)
        self.threads.append(t_image)
        t_image.start()

    # Gets the most current storage objects from the robotarms and returns them
    def get_objects(self) -> list[StorageObject]:
        # Retrieve updates from the robot arms
        for arm in self.robot_arms:
            for update in arm.get_object_updates():
                self._update_object(*update)

        return self.storage_objects

    # Tells the system to move an object to a given position
    def move_object(self, name: str, destination: str) -> None:
        with self.lock:
            obj = self._find_object_by_name(name)

            self.DT.create_event(("Pick Up", obj))

            # Tell to pick up from storage
            self.robot_arms[int(obj.position[-1])].add_to_queue(configuration["PickFromStoragePriority"], "Storage", obj.shape, obj.color)

            rules = self._make_rule_from_event(obj, destination)

            for i in range(len(self.robot_arms)):
                self.robot_arms[i].set_rules(rules[i])

            self.DT.set_rules(rules)

    # Retrive info from the DT
    # If anoamly that cannot be mitigated has occured then stop system
    def get_info_dt(self) -> dict[list, list, list]:
        info = self.DT.get_info_dt()

        for robot_id in info[0]["robots dropping object"]:
            self.robot_arms[robot_id].drop_object()

        for anomaly_log_object in info[1]:
            if anomaly_log_object[2] in ["Mitigation for anomaly 1, 3, 7, 8, 9 or 10 has failed", "Anomaly 14"]:
                self.stop_system()

        return info
