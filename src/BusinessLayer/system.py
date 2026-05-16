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
        self.stop_event = threading.Event()

        # Add all the robot arms
        for i in range(len(ips)):
            id_value = i
            ip_address = ips[i]
            poses = positions[i]
            self.robot_arms.append(RobotArm(ip_address, poses, id_value, copy.deepcopy(configuration["StorageOccupancy"][id_value]), self.stop_event))

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
            # if arm.ID == 1:
            #     print("START LOOP")

            arm.loop()

            # if arm.ID == 1:
            #     print("END LOOP")
            time.sleep(0.1)

        # Disconnect when not running
        arm.disconnect()
        print("Everything has been shut down")

    # Updates storage object to its new position
    def _update_object(self, shape: ObjectShape, color: ObjectColor, position: str) -> None:
        with self.lock:
            if position == "IR_0" or position == "IR_1":
                robot_id = int(position[-1])
                self.DT.create_event(("Object_seen_at_IR", (shape, color, robot_id)))

            for i in range(len(self.storage_objects)):
                obj = self.storage_objects[i]
                if obj.shape == shape and obj.color == color:
                    if position == "IR_0" or position == "IR_1":
                        position = "In_Transit"
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
                    if message[0] in ["Anomaly 13 Mitigation failed", "Anomaly 9"]:
                        robot_arm.set_mitigation_mode(True)
                        self._create_dt_anomaly_event(message[0], robot_id)
                    elif message[0] == "Anomaly 13":
                        id_value, shape, color = message[1]
                        self._create_dt_anomaly_event(message[0], id_value, shape, color)
                    elif message[0] == "Anomaly 3":
                        id_value, shape, color = message[1]
                        self._create_dt_anomaly_event(message[0], id_value, shape, color)
                        self._anomaly_3_mitigation(id_value, shape, color)
                    elif message[0] == "Anomaly 7":
                        id_value, shape, color = message[1]
                        self._create_dt_anomaly_event(message[0], id_value, shape, color)
                        self._anomaly_7_mitigation(id_value, shape, color)

            time.sleep(0.1)

    # Helper function to create the anomaly event in the DT
    def _create_dt_anomaly_event(self, eventype: str, robot_id: int = None, shape: ObjectShape = None, color: ObjectColor = None) -> None:
        event_param = (robot_id, shape, color)
        self.DT.create_event((eventype, event_param))

    # Create rules for the robotarms to mitigate anomaly 7
    def _anomaly_7_mitigation(self, robot_id_arrival: int, shape: ObjectShape, color: ObjectColor) -> None:
        # Rules for physical system
        self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})
        self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})
        self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
        self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

        # Rules for Digital twin
        self.DT.set_rules([None, {(shape, color): "Conveyor"}]) if robot_id_arrival else self.DT.set_rules([{(shape, color): "Conveyor"}, None])
        self.DT.set_rules([{(shape, color): "Storage"}, None]) if robot_id_arrival else self.DT.set_rules([None, {(shape, color): "Storage"}])

    # Create rules for the robotarms to mitigat anomaly 3
    def _anomaly_3_mitigation(self, robot_id_arrival: int, shape: ObjectShape, color: ObjectColor, add_to_queue: bool = True) -> None:
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

            if add_to_queue:
                self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
                self.robot_arms[robot_id_arrival].set_mitigation_mode(False)
        else:
            self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})
            self.DT.set_rules([None, {(shape, color): "Conveyor"}]) if robot_id_arrival else self.DT.set_rules([{(shape, color): "Conveyor"}, None])

            self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})
            self.DT.set_rules([{(shape, color): "Storage"}, None]) if robot_id_arrival else self.DT.set_rules([None, {(shape, color): "Storage"}])

            if add_to_queue:
                self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
                self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

    def _anomaly_8_mitigation(self, storage_id: int, shape: ObjectShape, color: ObjectColor) -> None:
        opposite_id = 0 if storage_id else 1
        self.robot_arms[opposite_id].remove_object_from_storage(shape, color)
        self.robot_arms[storage_id].add_to_queue(configuration["Anomaly8Priority"], "Storage", shape, color)
        self.robot_arms[opposite_id].set_rules({(shape, color): "Storage"})

    # Listens to the IR sensor, if the IR just switched from false to true, then create an event in the DT
    def _ir_listener(self) -> None:
        while self.running:
            with self.lock:
                self.DT.create_event(("IR", (self.robot_arms[0].get_ir(), self.robot_arms[1].get_ir())))

            time.sleep(0.1)

    def _image_listener(self) -> None:
        while self.running:
            for robot_id in range(len(self.robot_arms)):
                image, metadata = self.robot_arms[robot_id].get_latest_image()
                if image is not None and metadata is not None:
                    with self.lock:
                        event_param = (image, metadata)
                        self.DT.create_event(("Image", event_param))

            time.sleep(0.1)

    #
    #
    # Public functions
    #
    #

    # Stop the system from running, which means we cannot control the system anymore
    def stop_system(self) -> None:
        self.running = False
        self.stop_event.set()
        self.DT.stop_dt()

        # Wait for the threads to finnish their task before shutting down
        current_thread = threading.current_thread()
        if hasattr(self, "threads"):
            for t in self.threads:
                if t is not current_thread:
                    # If the thread doesn't close in 1s, move on anyway.
                    t.join(timeout=5.0)
                    if t.is_alive():
                        print(f"Warning: Thread {t.name} did not shut down cleanly.")

    # Setup all the listeners in system
    def set_up(self) -> None:
        self.DT.start_dt()

        self.threads = []

        t_ir = threading.Thread(target=self._ir_listener, daemon=True)
        self.threads.append(t_ir)
        t_ir.start()

        for arm in self.robot_arms:
            t = threading.Thread(target=self._robot_worker, args=(arm,), daemon=True)
            self.threads.append(t)
            t.start()

        t_anomaly = threading.Thread(target=self._anomaly_listener, daemon=True)
        self.threads.append(t_anomaly)
        t_anomaly.start()

        t_image = threading.Thread(target=self._image_listener, daemon=True)
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
    def get_info_dt(self):
        info = self.DT.get_info_dt()

        for robot_id in info[0]["robots dropping object"]:
            self.robot_arms[robot_id].drop_object()

        for anomaly_log_object in info[1]:
            print(f"{anomaly_log_object[2]}")
            if anomaly_log_object[2] in [
                "Anomaly 12 Mitigation failed",
                "Anomaly 13 Mitigation failed",
                configuration["Anomalies"][9],
                configuration["Anomalies"][14],
                configuration["Anomalies"][15],
                configuration["Anomalies"][5],
                "Either Anomaly 5, 10 or 11 has occured",
            ]:
                self.stop_system()

            elif anomaly_log_object[2] in [configuration["Anomalies"][8]]:
                storage_id = int(anomaly_log_object[1][-1])
                shape, color = anomaly_log_object[3]
                self._anomaly_8_mitigation(storage_id, shape, color)

            elif anomaly_log_object[2] in [configuration["Anomalies"][3]]:
                if len(anomaly_log_object) == 4:
                    shape, color, robot_id = anomaly_log_object[3]
                    self._anomaly_3_mitigation(robot_id, shape, color, False)

        storage_pickup_confirmation, robot_id = info[2]
        if storage_pickup_confirmation != "Waiting":
            self.robot_arms[robot_id].set_storage_pickup_confirmation(storage_pickup_confirmation)

        info_without_flags = (info[0], info[1])
        return info_without_flags
