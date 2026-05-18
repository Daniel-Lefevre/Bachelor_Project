from __future__ import annotations

import copy
import queue  # Safe thread queue
import threading
import time
from datetime import datetime
from typing import TYPE_CHECKING

# Import keyboard for the global hook listener
import keyboard

from resources.environment import StorageObject, configuration
from src.BusinessLayer.robot import RobotArm

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape


class System:
    def __init__(self, ips: list[str], positions: list[list[float]]):
        self.robot_arms = []
        self.running = True
        self.storage_objects = configuration["StorageObjects"]
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.anoamlies = []
        self.has_logged_anomaly_9 = False
        self.has_logged_anomaly_13 = False
        self.keyboard_queue = queue.Queue()

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

    def _keyboard_listener(self) -> None:
        """
        Low-level OS hook. Dumps actions instantly into a queue to prevent
        interfering with low-level thread timing or causing Unicode errors.
        """
        print("Thread-safe keyboard hooks active.")
        print("Press [k/m] for speed, [j] for direction, [h] for vacuum hold.")

        # Format: (arm_id, action_type, value)
        keyboard.on_press_key("k", lambda _: self.keyboard_queue.put((1, "speed", 15)))
        keyboard.on_press_key("m", lambda _: self.keyboard_queue.put((0, "speed", 15)))
        keyboard.on_press_key("j", lambda _: self.keyboard_queue.put((0, "direction", None)))
        keyboard.on_press_key("h", lambda _: self.keyboard_queue.put((1, "vacuum", None)))

    def _keyboard_processor_worker(self) -> None:
        """
        Dedicated background worker thread that pulls actions from the queue
        and applies them to the robot arms safely using the individual robot's lock.
        """
        while self.running:
            try:
                arm_id, action, value = self.keyboard_queue.get(timeout=0.1)

                if arm_id >= len(self.robot_arms):
                    self.keyboard_queue.task_done()
                    continue

                arm = self.robot_arms[arm_id]

                # CRITICAL FIX: Acquire the ROBOT's lock, not the System lock!
                # This prevents the keyboard thread from reading the TCP socket
                # while the robot worker thread is executing arm.loop()
                with arm.lock:
                    if action == "speed":
                        print(f"[KEYBOARD] Safely setting Robot {arm_id} conveyor speed to {value}")
                        arm.change_speed_of_conveyor_belt(value)
                    elif action == "direction":
                        print(f"[KEYBOARD] Safely changing Robot {arm_id} conveyor direction")
                        arm.change_conveyor_direction()
                    elif action == "vacuum":
                        print(f"[KEYBOARD] Safely locking Robot {arm_id} vacuum")
                        arm.dont_release_vacuum()

                self.keyboard_queue.task_done()

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error handling keyboard command: {e}")

    # Sets up the connections to the robot arms
    def _robot_worker(self, arm: RobotArm) -> None:
        # set up Phase
        print(f"Robot {arm.ID} is initializing")
        arm.set_up()
        print(f"Robot {arm.ID} setup finished")

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
                current_time = datetime.now().strftime("%H:%M")
                for message in messages:
                    if message[0] in ["Anomaly 13 Mitigation failed"]:
                        if not self.has_logged_anomaly_13:
                            self.anoamlies.append((current_time, f"Conveyor {robot_id}", "Anomaly 13 Mitigation failed"))
                        robot_arm.set_mitigation_mode(True)
                        self.has_logged_anomaly_13 = True

                        self.stop_system()
                    elif message[0] in ["Anomaly 9"]:
                        if not self.has_logged_anomaly_9:
                            self.anoamlies.append((current_time, f"Storage {robot_id}", configuration["Anomalies"][9]))
                        self.stop_system()
                        robot_arm.set_mitigation_mode(True)
                        self.has_logged_anomaly_9 = True
                    elif message[0] == "Anomaly 13":
                        id_value, shape, color = message[1]
                        self.anoamlies.append((current_time, f"Conveyor {robot_id}", configuration["Anomalies"][13]))
                    elif message[0] == "Anomaly 3":
                        id_value, shape, color = message[1]
                        self.anoamlies.append((current_time, f"Conveyor {robot_id}", configuration["Anomalies"][3]))
                        self._anomaly_3_mitigation(id_value, shape, color)
                    elif message[0] == "Anomaly 7":
                        id_value, shape, color = message[1]
                        self.anoamlies.append((current_time, f"Storage {robot_id}", configuration["Anomalies"][7]))
                        self._anomaly_7_mitigation(id_value, shape, color)

            time.sleep(0.1)

    # Create rules for the robotarms to mitigate anomaly 7
    def _anomaly_7_mitigation(self, robot_id_arrival: int, shape: ObjectShape, color: ObjectColor) -> None:
        # Rules for physical system
        self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})
        self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})
        self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
        self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

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

            if add_to_queue:
                self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
                self.robot_arms[robot_id_arrival].set_mitigation_mode(False)
        else:
            self.robot_arms[robot_id_arrival].set_rules({(shape, color): "Conveyor"})

            self.robot_arms[int(not robot_id_arrival)].set_rules({(shape, color): "Storage"})

            if add_to_queue:
                self.robot_arms[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
                self.robot_arms[robot_id_arrival].set_mitigation_mode(False)

    def _anomaly_8_mitigation(self, storage_id: int, shape: ObjectShape, color: ObjectColor) -> None:
        opposite_id = 0 if storage_id else 1
        self.robot_arms[opposite_id].remove_object_from_storage(shape, color)
        self.robot_arms[storage_id].add_to_queue(configuration["Anomaly8Priority"], "Storage", shape, color)
        self.robot_arms[opposite_id].set_rules({(shape, color): "Storage"})

    #
    #
    # Public functions
    #
    #

    # Stop the system from running, which means we cannot control the system anymore
    def stop_system(self) -> None:
        self.running = False
        self.stop_event.set()

        # Wait for the threads to finnish their task before shutting down
        current_thread = threading.current_thread()
        if hasattr(self, "threads"):
            for t in self.threads:
                if t is not current_thread:
                    # If the thread doesn't close in 5s, move on anyway.
                    t.join(timeout=5.0)
                    if t.is_alive():
                        print(f"Warning: Thread {t.name} did not shut down cleanly.")

    # Setup all the listeners in system
    def set_up(self) -> None:
        self.threads = []

        for arm in self.robot_arms:
            t = threading.Thread(target=self._robot_worker, args=(arm,), daemon=True)
            self.threads.append(t)
            t.start()

        t_anomaly = threading.Thread(target=self._anomaly_listener, daemon=True)
        self.threads.append(t_anomaly)
        t_anomaly.start()

        # New: Setup and start the keyboard hook worker thread
        self._keyboard_listener()

        t_kbd_processor = threading.Thread(target=self._keyboard_processor_worker, daemon=True)
        self.threads.append(t_kbd_processor)
        t_kbd_processor.start()

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

            # Tell to pick up from storage
            self.robot_arms[int(obj.position[-1])].add_to_queue(configuration["PickFromStoragePriority"], "Storage", obj.shape, obj.color)

            rules = self._make_rule_from_event(obj, destination)

            for i in range(len(self.robot_arms)):
                self.robot_arms[i].set_rules(rules[i])

    # Retrive info from the DT
    # If anoamly that cannot be mitigated has occured then stop system
    def get_anomalies(self):
        anomalies_copy = copy.deepcopy(self.anoamlies)
        self.anoamlies = []
        return anomalies_copy
