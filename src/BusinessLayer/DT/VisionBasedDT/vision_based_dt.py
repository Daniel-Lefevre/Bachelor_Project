from __future__ import annotations

from datetime import datetime
from queue import Queue
from types import SimpleNamespace
from typing import TYPE_CHECKING

from pyniryo import ObjectColor, ObjectShape

from resources.environment import configuration
from src.BusinessLayer.DT.States import ObjectStates
from src.BusinessLayer.DT.virtual_object import VirtualObject
from src.BusinessLayer.DT.virtual_robot import VirtualRobot

if TYPE_CHECKING:
    from resources.environment import StorageObject


class VisionBasedDT:
    def __init__(self, step_size: float):
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

    def get_state_snapshot(self) -> SimpleNamespace:
        objects = []
        for object in self.virtual_objects:
            object_info = SimpleNamespace(
                shape=object.shape,
                color=object.color,
                state=object.get_state(),
                progress=object.get_progress(),
                has_been_observed_on_conveyor=object.has_been_observed_on_conveyor,
                missing_counter=object.missing_counter,
            )
            objects.append(object_info)

        robots = []
        for robot in self.virtual_robots:
            robot_info = SimpleNamespace(robot_id=robot.id, state=robot.state)
            robots.append(robot_info)

        info = SimpleNamespace(objects=objects, robots=robots)

        return info

    def step(self, latest_ir_readings) -> None:
        # Check if something has arrived at ir
        for robot_id in range(len(self.virtual_robots)):
            if latest_ir_readings[robot_id] and not self.last_ir_readings[robot_id]:
                object_to_add = SimpleNamespace(state=SimpleNamespace(origin="IR"))
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

            elif event_type == "Storage_Pickup_Confirmation":
                storage_pickup_confirmation, robot_id = event_param
                self.virtual_robots[robot_id].set_storage_pickup_confirmation(storage_pickup_confirmation)

            elif event_type == "Update_DT":
                return_objects, location = event_param

                # Update all all the virtual objects in the DT based on the feedback from the image taken in vision module
                for return_object in return_objects:
                    for virtual_object in self.virtual_objects:
                        # Find the correct virtual object and check its state is conveyor
                        if return_object.color == virtual_object.color and return_object.shape == virtual_object.shape:
                            # Check of the return object is on the conveyor
                            if location == "Conveyors":
                                for update_category in return_object.updates:
                                    if update_category == "error_correction":
                                        # Update the virtual object
                                        error_correction = return_object.updates[update_category]
                                        virtual_object.set_progress(virtual_object.get_progress() + error_correction)
                                        if error_correction > 0:
                                            # Pushed backwards
                                            self.anomaly_log_messages.append((f"Conveyor {virtual_object.get_state().id}", "Either anomaly 1 or 4 has occured"))

                                        else:
                                            # Pushed forwards
                                            self.anomaly_log_messages.append((f"Conveyor {virtual_object.get_state().id}", "Either anomaly 2 or 4 has occured"))

                                    elif update_category == "has_been_observed":
                                        virtual_object.has_been_observed_on_conveyor = True

                                    elif update_category == "Anomaly_3":
                                        print(f"DT: {virtual_object.shape} {virtual_object.color}")
                                        virtual_object.set_state(f"Conveyor_{return_object.updates[update_category].id}")
                                        virtual_object.set_progress(return_object.updates[update_category].progress)

                                    elif update_category == "missing":
                                        virtual_object.missing_counter += 1

                            elif location == "Storage":
                                for update_category in return_object.updates:
                                    if update_category == "Updated_State":
                                        origin = return_object.updates[update_category].origin
                                        robot_id = return_object.updates[update_category].id
                                        opposite_robot_id = 0 if robot_id else 1
                                        state_key = f"{origin}_{robot_id}"

                                        if virtual_object.state.origin == "Robot" and virtual_object.state.id == robot_id:
                                            self.anomaly_log_messages.append((f"Storage {robot_id}", configuration["Anomalies"][12], (virtual_object.shape, virtual_object.color)))
                                        else:
                                            self.anomaly_log_messages.append((f"Storage {robot_id}", configuration["Anomalies"][8], (virtual_object.shape, virtual_object.color)))

                                            # Anomaly 8 Mitigation Plan
                                            self.virtual_robots[robot_id].add_to_queue(configuration["Anomaly8Priority"], virtual_object)
                                            new_rule = {(virtual_object.shape, virtual_object.color): "Storage"}
                                            self.virtual_robots[opposite_robot_id].set_rules(new_rule)
                                            self.virtual_robots[opposite_robot_id].storage.remove_object(virtual_object.shape, virtual_object.color)

                                        virtual_object.state = ObjectStates[state_key]

            elif event_type == "Anomaly 13":
                robot_id, shape, color = event_param
                self.anomaly_log_messages.append((robot_id, configuration["Anomalies"][13]))
                print(configuration["Anomalies"][13])
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

            elif event_type == "Anomaly 7":
                robot_id, shape, color = event_param
                self.anomaly_log_messages.append((f"Robot {robot_id}", configuration["Anomalies"][7]))
                print(configuration["Anomalies"][7])
                self.virtual_robots[robot_id].handle_anomaly(event_type)

            elif event_type == "Anomaly 9":
                robot_id, shape, color = event_param
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

            working_object, picked_up, placed_position, dropping_object, conveyor_turn_on = return_obj

            if conveyor_turn_on:
                for virtual_object in self.virtual_objects:
                    if virtual_object.state.origin == "Conveyor" and virtual_object.state.id == robot_id:
                        virtual_object.set_progress(virtual_object.get_progress() - 0.05)

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
            if len(log_message) == 2:
                actor, anomaly_text = log_message
                anomaly_logs.append((self._current_time(), actor, anomaly_text))
            elif len(log_message) == 3:
                actor, anomaly_text, extra_info = log_message
                anomaly_logs.append((self._current_time(), actor, anomaly_text, extra_info))

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
