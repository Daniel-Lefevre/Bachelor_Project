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

    def _object_to_virtual_object(self, object: StorageObject) -> VirtualObject:
        return VirtualObject(object.shape, object.color, self.step_size, int(object.position[-1]))

    def _find_virtual_object(self, shape: ObjectShape, color: ObjectColor) -> VirtualObject | None:
        for object in self.virtual_objects:
            if object.shape == shape and object.color == color:
                return object

    def step(self) -> None:
        conveyor_id_to_be_left = None
        # If an event has occured since last step
        while not self.events.empty():
            event_type, event_param = self.events.get()
            if event_type == "Pick Up":
                robot_id = int(event_param.state.id)
                self.virtual_robots[robot_id].add_to_queue(configuration["PickFromStoragePriority"], self._find_virtual_object(event_param.shape, event_param.color))
            elif event_type == "IR":
                conveyor_id_to_be_left = event_param
            else:
                print(f"Unknown Event: {event_type}")

        working_objects_info = []
        # Increment the time in all objects
        for i in range(len(self.virtual_robots)):
            virtual_robot = self.virtual_robots[i]
            working_object, pick_up_destination, placed_position = virtual_robot.step()
            if pick_up_destination is not None or placed_position is not None:
                working_objects_info.append((working_object, pick_up_destination, placed_position))

        for virtual_obj in self.virtual_objects:
            pick_up_destination = None
            placed_position = None
            for info in working_objects_info:
                if info[0] == virtual_obj:
                    pick_up_destination = info[1]
                    placed_position = info[2]

            ID = virtual_obj.state.id
            conveyor_running = self.virtual_conveyors[ID].get_info()
            virtual_obj.step(pick_up_destination, placed_position, conveyor_running, conveyor_id_to_be_left)
            # Check if virtual object has reached in IR sensor
            if virtual_obj.has_reached_ir:
                self.virtual_robots[ID].add_to_queue(configuration["PickFromIRSensorPriority"], virtual_obj)
                virtual_obj.has_reached_ir = False

    def create_event(self, event: tuple[str, int | StorageObject]) -> None:
        eventype, event_param = event
        if eventype == "Pick Up":
            self.events.put((eventype, self._object_to_virtual_object(event_param)))
        elif eventype == "IR":
            self.events.put((eventype, event_param))

    def set_rules(self, rules: list[dict]) -> None:
        # Set the rules on the virtual robot arms
        for i in range(len(self.virtual_robots)):
            self.virtual_robots[i].set_rules(rules[i])

    def get_info_dt(self) -> dict[str, list]:
        info = {"robots": [], "objects": []}

        for robot in self.virtual_robots:
            info["robots"].append(robot.get_info())

        for obj in self.virtual_objects:
            info["objects"].append(obj.get_info())

        return info
