from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import keyboard

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

        # Add all the robot arms
        for i in range(len(ips)):
            id_value = i
            ip_address = ips[i]
            poses = positions[i]
            self.robot_arms.append(RobotArm(ip_address, poses, id_value))

    def stop_system(self) -> None:
        print("STOP")
        self.running = False
        self.DT.running = False

    def set_up(self) -> None:
        self.DT.start_dt()

        self.threads = []

        t_ir = threading.Thread(target=self.ir_listener)
        self.threads.append(t_ir)
        t_ir.start()

        for arm in self.robot_arms:
            t = threading.Thread(target=self.robot_worker, args=(arm,))
            self.threads.append(t)
            t.start()

        t_anomaly = threading.Thread(target=self.anomaly_listener)
        self.threads.append(t_anomaly)
        t_anomaly.start()

        self.startup_robots_loop()

    def robot_worker(self, arm: RobotArm) -> None:
        # set up Phase
        print(f"Robot {arm.ID} is initializing")
        arm.set_up()
        print(f"Robot {arm.ID} setup finished")

        # 2. Monitoring Phase
        while self.running:
            arm.loop()
            time.sleep(0.05)

        # Disconnect when not running
        arm.disconnect()
        print("Everything has been shut down")

    def startup_robots_loop(self) -> None:
        while self.running:
            # Robot 0 take image
            if keyboard.is_pressed("9"):
                self.robot_arms[0].take_image()
                time.sleep(0.5)

            # Robot 1 take image
            elif keyboard.is_pressed("2"):
                self.robot_arms[1].take_image()
                time.sleep(0.5)

            time.sleep(0.05)

        # Wait for the threads to finnish their task before shutting down
        for t in self.threads:
            t.join()

    def update_object(self, shape: ObjectShape, color: ObjectColor, position: str) -> None:
        with self.lock:
            for i in range(len(self.storage_objects)):
                obj = self.storage_objects[i]
                if obj.shape == shape and obj.color == color:
                    self.storage_objects[i].position = position

    def get_objects(self) -> list[StorageObject]:
        # Retrieve updates from the robot arms
        for arm in self.robot_arms:
            for update in arm.get_object_updates():
                self.update_object(*update)

        return self.storage_objects

    def find_object_by_name(self, object_name: str) -> StorageObject:
        for object in self.storage_objects:
            if object.name == object_name:
                return object

    def move_object(self, name: str, destination: str) -> None:
        with self.lock:
            obj = self.find_object_by_name(name)

            self.DT.create_event(("Pick Up", obj))

            # Tell to pick up from storage
            self.robot_arms[int(obj.position[-1])].add_to_queue(configuration["PickFromStoragePriority"], "Storage", obj.shape, obj.color)

            rules = self.make_rule_from_event(obj, destination)

            for i in range(len(self.robot_arms)):
                self.robot_arms[i].set_rules(rules[i])

            self.DT.set_rules(rules)

    def make_rule_from_event(self, object: StorageObject, destination: str) -> list[dict]:
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

    def anomaly_listener(self) -> None:
        while self.running:
            for robot_arm in self.robot_arms:
                messages = robot_arm.get_anomaly_updates()
                for message in messages:
                    if message[0] == "Stop System":
                        self.stop_system()
                        print("Human Intervention Required")
                    elif message[0] == "Anomaly 5":
                        id_value, shape, color = message[1]
                        self.anomaly_5_mitigation(id_value, shape, color)

            time.sleep(0.1)

    def anomaly_5_mitigation(self, robot_id_arrival: int, shape: ObjectShape, color: ObjectColor) -> None:
        print(f"ID: {robot_id_arrival}")
        goal_storage_id = None
        for storage_object in self.storage_objects:
            if storage_object.shape == shape and storage_object.color == color:
                if storage_object.position == "In_Transit":
                    for robot_id in range(len(self.robot_arms)):
                        if self.robot_arms[robot_id].get_rules().get((shape, color)) == "Storage":
                            goal_storage_id = robot_id
                else:
                    print("happend")
                    goal_storage_id = int(storage_object.position[-1])
                    print(goal_storage_id)
                break
        if goal_storage_id == robot_id_arrival:
            print("TRUE")
            self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Storage"})
            self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
            self.robot_arms[robot_id_arrival].set_mitigation_mode(False)
        else:
            print("False")
            self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})
            self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})
            self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
            self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

    def ir_listener(self) -> None:
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

    def get_info_dt(self) -> dict[str, list]:
        return self.DT.get_info_dt()
