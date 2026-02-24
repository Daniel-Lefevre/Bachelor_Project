from __future__ import annotations

from queue import Queue
from typing import TYPE_CHECKING

from pyniryo import ObjectColor, ObjectShape

from resources.environment import configuration
from src.BusinessLayer.DT.virtual_conveyor import VirtualConveyor
from src.BusinessLayer.DT.virtual_object import VirtualObject
from src.BusinessLayer.DT.virtual_robot import VirtualRobot

if TYPE_CHECKING:
    from resources.environment import StorageObject


class TimeBasedDT:
    def __init__(self, step_size: int):
        self.step_size = step_size
        self.virtual_objects = [self._object_to_virtual_object(obj) for obj in configuration["StorageObjects"]]
        self.events = Queue()
        self.virtual_conveyors = [VirtualConveyor(id) for id in range(configuration["NumberOfConveyors"])]
        self.virtual_robots = [VirtualRobot(id, self.step_size, self.virtual_conveyors[id]) for id in range(configuration["NumberOfRobotArms"])]
        self.robots_dropping_objects = []

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

    def _check_virtual_objects_at_ir_sensor(self, conveyor_id: int) -> bool:
        for virt_obj in self.virtual_objects:
            if virt_obj.get_state().key == f"IR_{conveyor_id}":
                return True

        return False

    def step(self) -> None:
        # print("----------")
        # If an event has occured since last step
        leaving_conveyor = None
        while not self.events.empty():
            event_type, event_param = self.events.get()
            if event_type == "Pick Up":
                robot_id = int(event_param.state.id)
                self.virtual_robots[robot_id].add_to_queue(configuration["PickFromStoragePriority"], self._find_virtual_object(event_param.shape, event_param.color))
            elif event_type == "IR":
                conveyor_id = event_param
                furthest_object = self._get_object_furthest_on_conveyor(conveyor_id)
                if furthest_object is not None:
                    leaving_conveyor = (furthest_object.shape, furthest_object.color, conveyor_id)

            elif event_type == "Setup done":
                for robot in self.virtual_robots:
                    robot.exit_setup()

            elif event_type == "Anomaly 4":
                print(configuration["Anomalies"][4])
                robot_id, shape, color = event_param
                self.virtual_robots[robot_id].handle_anomaly(event_type)
                for virt_obj in self.virtual_objects:
                    if virt_obj.shape == shape and virt_obj.color == color:
                        virt_obj.handle_anomaly(event_type)

            elif event_type == "Anomaly 4 Mitigation failed":
                print("Anomaly 4 Mitigation failed, Human intervention required")

            elif event_type == "Anomaly 5":
                print(configuration["Anomalies"][5])
                robot_id_arrival, shape, color = event_param
                for virt_obj in self.virtual_objects:
                    if virt_obj.shape == shape and virt_obj.color == color:
                        virt_obj.handle_anomaly("Anomaly 5", robot_id_arrival)
                        self.virtual_robots[robot_id_arrival].add_to_queue(configuration["PickFromIRSensorPriority"], virt_obj)

            elif event_type == "Anomaly 11":
                print(configuration["Anomalies"][11])
                robot_id, shape, color = event_param
                self.virtual_robots[robot_id].handle_anomaly(event_type)
                # for virt_obj in self.virtual_objects:
                #     if virt_obj.shape == shape and virt_obj.color == color:
                #         self.virtual_robots[robot_id].add_to_queue(configuration["PickFromIRSensorPriority"], virt_obj)

            elif event_type == "Anomaly 14":
                print(configuration["Anomalies"][14])

            else:
                print(f"Unknown Event: {event_type}")

        working_objects_info = []
        # Increment the time in all objects
        for robot_id in range(len(self.virtual_robots)):
            virtual_robot = self.virtual_robots[robot_id]

            print(f"Robot {robot_id}: {virtual_robot.state}")

            # Check if there is an object in the robots drop off zone on the conveyor
            conveyor_id = int(not robot_id)
            object_at_drop_off = self._check_virtual_objects_drop_off_state(conveyor_id)

            object_at_ir = self._check_virtual_objects_at_ir_sensor(robot_id)
            return_obj = virtual_robot.step(object_at_drop_off, object_at_ir)
            # print(f"Robot {robot_id} state: {virtual_robot.state.key}")
            if return_obj is None:
                return

            working_object, picked_up, placed_position, dropping_object = return_obj
            if dropping_object:
                self.robots_dropping_objects.append(robot_id)
            if picked_up or placed_position is not None:
                working_objects_info.append((working_object, picked_up, placed_position))

        for virtual_obj in self.virtual_objects:
            if virtual_obj.color == ObjectColor.GREEN and virtual_obj.shape == ObjectShape.CIRCLE:
                print(virtual_obj.state)
                print("-------------")
            picked_up = None
            placed_position = None
            for info in working_objects_info:
                if info[0] == virtual_obj:
                    picked_up = info[1]
                    placed_position = info[2]

            ID = virtual_obj.state.id
            conveyor_running = self.virtual_conveyors[ID].get_info()

            conveyor_id_to_be_left = None
            if leaving_conveyor is not None and leaving_conveyor[0] == virtual_obj.shape and leaving_conveyor[1] == virtual_obj.color:
                conveyor_id_to_be_left = leaving_conveyor[2]

            virtual_obj.step(picked_up, placed_position, conveyor_running, conveyor_id_to_be_left)
            # if virtual_obj.color == ObjectColor.RED and virtual_obj.shape == ObjectShape.CIRCLE:
            # print(f"RED Circle state: {virtual_obj.state.key}")
            # Check if virtual object has reached in IR sensor

            if virtual_obj.get_ir_state():
                self.virtual_robots[ID].add_to_queue(configuration["PickFromIRSensorPriority"], virtual_obj)
                virtual_obj.set_ir_state(False)

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

    def get_info_dt(self) -> dict[list, list, list]:
        info = {"robots": [], "objects": [], "robots dropping object": []}

        for robot in self.virtual_robots:
            info["robots"].append(robot.get_info())

        for obj in self.virtual_objects:
            info["objects"].append(((obj.shape, obj.color), obj.get_info()))

        info["robots dropping object"] = self.robots_dropping_objects.copy()
        self.robots_dropping_objects = []

        return info
