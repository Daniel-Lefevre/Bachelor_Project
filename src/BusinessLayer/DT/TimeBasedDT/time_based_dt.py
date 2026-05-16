from __future__ import annotations

from datetime import datetime
from queue import Queue
from types import SimpleNamespace
from typing import TYPE_CHECKING

from pyniryo import ObjectColor, ObjectShape

from resources.environment import configuration
from src.BusinessLayer.DT.virtual_object import VirtualObject
from src.BusinessLayer.DT.virtual_robot import VirtualRobot

if TYPE_CHECKING:
    from resources.environment import StorageObject


class TimeBasedDT:
    def __init__(self, step_size: int):
        self.step_size = step_size
        self.virtual_objects = [self._object_to_virtual_object(obj) for obj in configuration["StorageObjects"]]
        self.events = Queue()
        self.virtual_robots = [VirtualRobot(id, self.step_size) for id in range(configuration["NumberOfRobotArms"])]
        self.robots_dropping_objects = []
        self.anomaly_log_messages = []
        self.last_ir_readings = (False, False)

    #
    #
    # Private function
    #
    #

    def _object_to_virtual_object(self, object: StorageObject) -> VirtualObject:
        return VirtualObject(object.shape, object.color, self.step_size, int(object.position[-1]))

    def _find_virtual_object(self, shape: ObjectShape, color: ObjectColor) -> VirtualObject | None:
        for object in self.virtual_objects:
            if object.shape == shape and object.color == color:
                return object

    def _check_virtual_objects_drop_off_state(self, conveyor_id: int) -> bool:
        for virt_obj in self.virtual_objects:
            if virt_obj.is_at_drop_off(conveyor_id):
                return True

        return False

    #
    #
    # Public function
    #
    #

    def step(self, latest_ir_readings) -> None:
        # Check if something has arrived at ir
        for robot_id in range(len(self.virtual_robots)):
            if latest_ir_readings[robot_id] and not self.last_ir_readings[robot_id]:
                highest_progress = 0
                highest_progress_object = None

                for virtual_object in self.virtual_objects:
                    if virtual_object.get_progress() > highest_progress:
                        highest_progress = virtual_object.get_progress()
                        highest_progress_object = virtual_object

                object_to_add = SimpleNamespace(state=SimpleNamespace(origin="IR", shape=highest_progress_object.shape, color=highest_progress_object.color))
                self.virtual_robots[robot_id].add_to_queue(configuration["PickFromIRSensorPriority"], object_to_add)

        # If an event has occured since last step
        while not self.events.empty():
            event_type, event_param = self.events.get()
            if event_type == "Pick Up":
                robot_id = int(event_param.state.id)
                self.virtual_robots[robot_id].add_to_queue(configuration["PickFromStoragePriority"], self._find_virtual_object(event_param.shape, event_param.color))

            elif event_type == "Setup done":
                for robot in self.virtual_robots:
                    robot.exit_setup()

            elif event_type == "Object_seen_at_IR":
                shape, color, robot_id = event_param
                for virtual_object in self.virtual_objects:
                    if virtual_object.shape == shape and virtual_object.color == color:
                        virtual_object.set_ir_hit_progress(robot_id)
                        virtual_object.set_state(f"IR_{robot_id}")

                self.virtual_robots[robot_id].set_working_object(self._find_virtual_object(shape, color))

            elif event_type == "Anomaly 13":
                robot_id, shape, color = event_param
                self.anomaly_log_messages.append((robot_id, configuration["Anomalies"][13]))
                print(configuration["Anomalies"][13])
                print(robot_id)
                self.virtual_robots[robot_id].handle_anomaly(event_type)
                for virt_obj in self.virtual_objects:
                    if virt_obj.shape == shape and virt_obj.color == color:
                        virt_obj.handle_anomaly(event_type)

            elif event_type == "Anomaly 13 Mitigation failed":
                robot_id = event_param
                self.anomaly_log_messages.append((f"Robot {robot_id}", "Anomaly 13 Mitigation failed"))
                print("Anomaly 13 Mitigation failed, Human intervention required")

            elif event_type == "Anomaly 3":
                robot_id_arrival, shape, color = event_param
                self.anomaly_log_messages.append((f"Robot {robot_id_arrival}", configuration["Anomalies"][3]))
                print(configuration["Anomalies"][3])
                for virt_obj in self.virtual_objects:
                    if virt_obj.shape == shape and virt_obj.color == color:
                        virt_obj.handle_anomaly("Anomaly 3", robot_id_arrival)
                self.virtual_robots[robot_id_arrival].set_working_object(self._find_virtual_object(shape, color))

            elif event_type == "Anomaly 7":
                robot_id, shape, color = event_param
                self.anomaly_log_messages.append((f"Robot {robot_id}", configuration["Anomalies"][7]))
                print(configuration["Anomalies"][7])
                self.virtual_robots[robot_id].handle_anomaly(event_type)

            elif event_type == "Anomaly 9":
                robot_id = event_param
                self.anomaly_log_messages.append((f"Robot {robot_id}", configuration["Anomalies"][9]))
                print(configuration["Anomalies"][9])

            else:
                print(f"Unknown Event: {event_type}")

        working_objects_info = []
        # Increment the time in all objects
        for robot_id in range(len(self.virtual_robots)):
            virtual_robot = self.virtual_robots[robot_id]

            # Check if there is an object in the robots drop off zone on the conveyor
            conveyor_id = int(not robot_id)
            object_at_drop_off = self._check_virtual_objects_drop_off_state(conveyor_id)

            object_at_ir = latest_ir_readings[robot_id]
            return_obj = virtual_robot.step(object_at_drop_off, object_at_ir)
            # print(f"Robot {robot_id} state: {virtual_robot.state.key}")
            if return_obj is None:
                return

            working_object, picked_up, placed_position, dropping_object, _ = return_obj
            if dropping_object:
                self.robots_dropping_objects.append(robot_id)
            if picked_up or placed_position is not None:
                working_objects_info.append((working_object, picked_up, placed_position))

        for virtual_obj in self.virtual_objects:
            # if virtual_obj.color == ObjectColor.GREEN and virtual_obj.shape == ObjectShape.CIRCLE:
            # print(virtual_obj.state)
            # print("-------------")
            picked_up = None
            placed_position = None
            for info in working_objects_info:
                if info[0] == virtual_obj:
                    picked_up = info[1]
                    placed_position = info[2]

            ID = virtual_obj.state.id
            conveyor_running = self.virtual_robots[ID].get_conveyor_info()

            virtual_obj.step(picked_up, placed_position, conveyor_running)
            # if virtual_obj.color == ObjectColor.RED and virtual_obj.shape == ObjectShape.CIRCLE:
            # print(f"RED Circle state: {virtual_obj.state.key}")
            # Check if virtual object has reached in IR sensor

        self.last_ir_readings = latest_ir_readings

    def _get_object_furthest_on_conveyor(self, conveyor_id: int) -> None | VirtualObject:
        current_furthest_object = None
        current_furthest_distance = -float("inf")
        for obj in self.virtual_objects:
            # Check object on correct conveyor
            if obj.get_state().key == f"Conveyor_{conveyor_id}":
                # Check object is furthest
                if obj.get_progress() > current_furthest_distance:
                    current_furthest_object = obj
                    current_furthest_distance = obj.get_progress()
        return current_furthest_object

    def create_event(self, event: tuple[str, int | StorageObject]) -> None:
        eventype, event_param = event
        if eventype == "Pick Up":
            self.events.put((eventype, self._object_to_virtual_object(event_param)))
        else:
            self.events.put((eventype, event_param))

    def set_rules(self, rules: list[dict]) -> None:
        # Set the rules on the virtual robot arms
        for i in range(len(self.virtual_robots)):
            if rules[i] is None:
                continue
            self.virtual_robots[i].set_rules(rules[i])

    def _current_time(self):
        return datetime.now().strftime("%H:%M")

    def get_info_dt(self) -> tuple[list[tuple[str, int, str]], dict[list, list, list]]:
        animation_info = {"robots": [], "objects": [], "robots dropping object": []}
        anomaly_logs = []

        for log_message in self.anomaly_log_messages:
            actor, anomaly_text = log_message
            anomaly_logs.append((self._current_time(), actor, anomaly_text))

        for robot in self.virtual_robots:
            info, robot_anomaly_logs = robot.get_info()
            animation_info["robots"].append(info)
            for log in robot_anomaly_logs:
                actor, anomaly_text = log
                anomaly_logs.append((self._current_time(), actor, anomaly_text))

        for obj in self.virtual_objects:
            storage_index = self.virtual_robots[obj.state.id].get_storage_position(obj.shape, obj.color)
            info = obj.get_info()
            animation_info["objects"].append(((obj.shape, obj.color), info[0], storage_index))
            for log_message in info[1]:
                actor, anomaly_text = log_message
                anomaly_logs.append((self._current_time(), actor, anomaly_text))

        animation_info["robots dropping object"] = self.robots_dropping_objects.copy()
        self.robots_dropping_objects = []

        self.anomaly_log_messages = []

        return (animation_info, anomaly_logs)
