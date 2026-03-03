from __future__ import annotations

import copy
from typing import TYPE_CHECKING

from resources.environment import configuration
from resources.PriorityQueue import CustomPriorityQueue
from src.BusinessLayer.DT.states import create_robot_states
from src.BusinessLayer.DT.virtual_conveyor import VirtualConveyor
from src.BusinessLayer.DT.virtual_storage import VirtualStorage

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape

    from src.BusinessLayer.DT.states import RobotStates
    from src.BusinessLayer.DT.virtual_object import VirtualObject


class VirtualRobot:
    def __init__(self, id: int, step_size: int):
        self.rules = {}
        self.queue = CustomPriorityQueue(configuration["NumberOfPriorities"])
        self.id = id
        self.states = create_robot_states(self.id)
        self.state = self.states["Setup"]
        self.step_size = step_size
        self.current_state_progress_goal = float("inf")
        self.current_state_progress = 0
        self.working_object = None
        self.conveyor = VirtualConveyor(self.id)
        self.storage = VirtualStorage(self.id)
        self.next_destination = None
        self.anomaly_logs = []
        self.has_exited_anoamly15 = True

    # Handles the transition between states of the virtual robot arm
    def _state_transition(self, objec_at_drop_off: bool) -> tuple[bool, str | None]:
        picked_up = False
        placed_position = None
        dropping_object = False

        # In this state the robot picks up an object and therefore sets the flag and removes it from storage
        if self.state.key == "Observation_to_Pickup_Storage":
            self.state = self.states["Storage_to_Standby"]
            self.storage.remove_object(self.working_object.shape, self.working_object.color)
            picked_up = True

        #
        elif self.state.key in ["Storage_to_Standby", "Observation_to_Standby"]:
            if objec_at_drop_off:
                if self.has_exited_anoamly15:
                    self.anomaly_logs.append((f"Robot {self.id}", "Anomaly 15 has occured"))
                self.state = self.states["Standby"]
                self.has_exited_anoamly15 = False
            else:
                self.state = self.states["Standby_to_Place_Conveyor"]
                dropping_object = True
                self.has_exited_anoamly15 = True

        elif self.state.key == "Standby_to_Place_Conveyor":
            placed_position = "Conveyor"
            self.state = self.states["Place_Conveyor_to_Observation"]

        elif self.state.key in ["Place_Conveyor_to_Observation", "Place_Storage_to_Observation"]:
            self.state = self.states["Observation"]

        elif self.state.key == "Observation_to_Pickup_Conveyor":
            self.state = self.states["Pickup_Conveyor_to_Observation"]
            picked_up = True

        elif self.state.key == "Pickup_Conveyor_to_Observation":
            next_destination = self.rules[(self.working_object.shape, self.working_object.color)]
            if next_destination == "Conveyor":
                self.state = self.states["Observation_to_Standby"]

            elif next_destination == "Storage":
                self.state = self.states["Observation_to_Place_Storage"]

        elif self.state.key == "Observation_to_Place_Storage":
            self.state = self.states["Place_Storage_to_Observation"]
            placed_position = "Storage"
            self.storage.add_object(self.working_object.shape, self.working_object.color)

        self.current_state_progress = 0
        self.current_state_progress_goal = self.state.time

        return (picked_up, placed_position, dropping_object)

    def get_conveyor_info(self) -> bool:
        return self.conveyor.get_info()

    def get_storage_position(self, shape: ObjectShape, color: ObjectColor) -> int | None:
        return self.storage.get_storage_position(shape, color)

    def set_rules(self, rules: dict) -> None:
        for rule_key in rules:
            self.rules[rule_key] = rules[rule_key]

    def add_to_queue(self, priority: int, virtual_object: VirtualObject | None) -> None:
        self.queue.put((priority, virtual_object))

    def handle_anomaly(self, anomaly: str):
        if anomaly == "Anomaly 4":
            if self.state.key in ["Pickup_Conveyor_to_Observation", "Observation_to_Standby", "Observation_to_Place_Storage"]:
                self.state = self.states["Observation_to_Pickup_Conveyor"]
                self.current_state_progress = 0
                self.current_state_progress_goal = self.state.time
            else:
                raise Exception(f"Wrong state for anomaly 4 {self.state}")
        elif anomaly == "Anomaly 11":
            if self.state.key in ["Place_Storage_to_Observation", "Place_Conveyor_to_Observation", "Observation", "Observation_to_Pickup_Conveyor"]:
                self.state = self.states["Observation_to_Pickup_Conveyor"]
                self.current_state_progress = 0
                self.current_state_progress_goal = self.state.time
            else:
                print(self.state.key)
                raise Exception(f"Wrong state for anomaly 11 {self.state}")
        else:
            raise Exception(f"Unknown anomaly: {anomaly}")

    def step(self, objec_at_drop_off: bool, object_at_ir: bool) -> tuple[VirtualObject | None, str | None, bool | None] | None:
        if self.id == 0:
            print(f"status: {self.conveyor.get_info()}")
        # If in setup dont do anything
        if self.state.key == "Setup":
            return None

        if self.state.key == "Observation" and not self.queue.empty():
            self.working_object = self.queue.get()
            destination = "Conveyor" if self.working_object.state.origin == "IR" else self.working_object.state.origin
            self.state = self.states[f"Observation_to_Pickup_{destination}"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0

        dropping_object = False

        # If Robot is in standby and the drop off zone is clear
        if self.state.key == "Standby" and not objec_at_drop_off:
            self.state = self.states["Standby_to_Place_Conveyor"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0
            dropping_object = True
            self.has_exited_anoamly15 = True

        picked_up = False
        placed_position = None

        if self.current_state_progress >= self.current_state_progress_goal:
            picked_up, placed_position, dropping_object_ret = self._state_transition(objec_at_drop_off)
            dropping_object = dropping_object or dropping_object_ret
            self.current_state_progress = 0

        if self.state.key in ["Observation", "Standby"] and not object_at_ir:
            self.conveyor.start()
        else:
            self.conveyor.stop()

        self.current_state_progress += self.step_size

        # print(f"{self.id}{self.state.key}: {picked_up}")
        return (self.working_object, picked_up, placed_position, dropping_object)

    def get_info(self) -> tuple[RobotStates, float]:
        progress = 0
        if self.current_state_progress_goal == float("inf"):
            progress = float("inf")
        elif self.current_state_progress > self.current_state_progress_goal:
            progress = 1
        else:
            progress = self.current_state_progress / self.current_state_progress_goal
        animation_info = (self.state, progress)
        anomaly_logs = copy.deepcopy(self.anomaly_logs)
        self.anomaly_logs = []
        return (animation_info, anomaly_logs)

    def exit_setup(self) -> None:
        self.state = self.states["Observation"]
