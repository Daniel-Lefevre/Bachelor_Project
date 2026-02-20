from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from resources.environment import configuration
from resources.PriorityQueue import CustomPriorityQueue
from src.BusinessLayer.DT.states import create_robot_states

if TYPE_CHECKING:
    from src.BusinessLayer.DT.states import RobotStates
    from src.BusinessLayer.DT.virtual_conveyor import VirtualConveyor
    from src.BusinessLayer.DT.virtual_object import VirtualObject


class VirtualRobot:
    def __init__(self, id: int, step_size: int, conveyor: VirtualConveyor):
        self.rules = {}
        self.queue = CustomPriorityQueue(configuration["NumberOfPriorities"])
        self.id = id
        self.states = create_robot_states(self.id)
        self.state = self.states["Observation"]
        self.step_size = step_size
        self.current_state_progress_goal = float("inf")
        self.current_state_progress = 0
        self.working_object = None
        self.conveyor = conveyor

    def set_rules(self, rules: dict) -> None:
        for rule_key in rules:
            self.rules[rule_key] = rules[rule_key]

    def add_to_queue(self, priority: int, virtual_object: VirtualObject | None) -> None:
        self.queue.put((priority, virtual_object))

    def step(self, objec_at_drop_off: bool) -> tuple[VirtualObject | None, str | None, Literal["Conveyor", "Storage"] | None]:
        if self.state.key == "Observation" and not self.queue.empty():
            self.working_object = self.queue.get()
            destination = "Conveyor" if self.working_object.state.origin == "IR" else self.working_object.state.origin
            self.state = self.states[f"Observation_to_{destination}"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0

        # If Robot is in standby and the drop off zone is clear
        if self.state.key == "Standby" and not objec_at_drop_off:
            self.state = self.states["Standby_to_Conveyor"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0

        destination = None
        placed_position = None

        if self.current_state_progress >= self.current_state_progress_goal:
            destination, placed_position = self._state_transition(objec_at_drop_off)
            self.current_state_progress = 0

        self.current_state_progress += self.step_size

        if self.state.key == "Observation":
            self.conveyor.start()
        else:
            self.conveyor.stop()

        return (self.working_object, destination, placed_position)

    # def handle_anomaly(self, anomaly: str):
    # if anomaly == "Anomaly 4":
    # if (self.state.key in ["Storage_to_Observation", "Storage_to_Observation_place"])

    # self.state = self.states[state]
    # self.current_state_progress = 0
    # self.current_state_progress_goal = self.state.time

    def _state_transition(self, objec_at_drop_off: bool) -> tuple[str | None, Literal["Conveyor", "Storage"] | None]:
        destination = None
        placed_position = None

        if self.state.key == "Observation_to_Storage":
            destination = "Standby"
            self.state = self.states[f"Storage_to_{destination}"]

        elif self.state.key == "Observation_to_Conveyor":
            destination = self.rules.get((self.working_object.shape, self.working_object.color))
            destination = "Standby" if destination == "Conveyor" else "Observation"
            self.rules.pop((self.working_object.shape, self.working_object.color))
            self.state = self.states[f"Conveyor_to_{destination}"]

        elif self.state.key == "Storage_to_Standby" or self.state.key == "Conveyor_to_Standby":
            if objec_at_drop_off:
                self.state = self.states["Standby"]
                print("Anomaly 15, drop off is obstructed")
            else:
                self.state = self.states["Standby_to_Conveyor"]

        elif self.state.key == "Standby_to_Conveyor":
            self.state = self.states["Conveyor_to_Observation"]
            placed_position = "Conveyor"

        elif self.state.key == "Storage_to_Observation":
            self.state = self.states["Observation"]

        elif self.state.key == "Conveyor_to_Storage":
            self.state = self.states["Storage_to_Observation"]
            placed_position = "Storage"

        elif self.state.key == "Conveyor_to_Observation":
            self.state = self.states["Observation"]

        self.current_state_progress_goal = self.state.time

        return (destination, placed_position)

    def get_info(self) -> tuple[RobotStates, float]:
        return (self.state, self.current_state_progress / self.current_state_progress_goal if self.current_state_progress_goal != float("inf") else float("inf"))
