from __future__ import annotations

import copy
import time
from typing import TYPE_CHECKING

from src.BusinessLayer.DT.States import ObjectStates

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape

    from src.BusinessLayer.DT.States import ObjectState


class VirtualObject:
    def __init__(self, shape: ObjectShape, color: ObjectColor, step_size: int, starting_storage_id: int):
        self.states = ObjectStates
        self.state = self.states[f"Storage_{starting_storage_id}"]
        self.color = color
        self.shape = shape
        self.current_state_progress_goal = float("inf")
        self.current_state_progress = 0
        self.step_size = step_size
        self.has_reached_ir = False
        self.anomaly_logs = []
        self.time_object_went_missing = None

    # PUBLIC METHODS
    def set_ir_state(self, state: bool) -> None:
        self.has_reached_ir = state

    def get_ir_state(self) -> bool:
        return self.has_reached_ir

    def get_state(self):
        return self.state

    def get_progress(self) -> float:
        return self.current_state_progress / self.current_state_progress_goal

    # Checks if the object is inside the drop-off zone
    def is_at_drop_off(self, conveyor_id: int) -> bool:
        if self.state.key == f"Conveyor_{conveyor_id}":
            return (self.current_state_progress / self.current_state_progress_goal) < 1 / 6

        else:
            return False

    # Handles the different anomaly cases for the virutal object
    def handle_anomaly(self, anomaly: str, params=None):
        if anomaly == "Anomaly 4":
            # If the robot arm fails to pick up the object from the conveyor, then the state is set back down to the IR sensor
            if self.state.origin == "Robot":
                self.state = self.states[f"IR_{self.state.id}"]
                self.current_state_progress = 0
                self.current_state_progress_goal = self.state.time
            else:
                raise Exception(f"Wrong state for anomaly 4: {self.state}")

        elif anomaly == "Anomaly 5":
            # if the wrong object is detected by a robot, then the found objects state should be set down to the IR sensor where it appeared
            robot_arrival_id = params
            self.state = self.states[f"IR_{robot_arrival_id}"]
            self.current_state_progress = 0
            self.current_state_progress_goal = self.state.time

        else:
            raise Exception(f"Unknown anomaly: {anomaly}")

    # Simulate a single step in the virtual object
    def step(self, picked_up: bool, placed_position: str | None, conveyor_running: bool, activated_ir_id: int | None):
        # if the virtual object has reached an IR sensor
        if activated_ir_id is not None and self.state.origin == "Conveyor" and self.state.id == activated_ir_id:
            # Check if the object has arrived too early
            if self.current_state_progress_goal - self.current_state_progress > 1.0:
                self.anomaly_logs.append((f"Conveyor {self.state.id}", "Either anomaly 2 or 7 has occured"))
            # Check if the object has arrived too late
            elif self.current_state_progress_goal - self.current_state_progress < -1.0:
                self.anomaly_logs.append((f"Conveyor {self.state.id}", "Either anomaly 1 or 7 has occured"))
                self.time_object_went_missing = None

            # Since either anomaly group has the mitigation to continue, then continue
            self.state = self.states[f"IR_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0
            self.has_reached_ir = True

        # if the object has not reached an IR sensor and is arriving late then a larger group of anomalies has occured
        elif self.current_state_progress_goal - self.current_state_progress < -1.0:
            if self.time_object_went_missing is None:
                self.anomaly_logs.append((f"Conveyor {self.state.id}", "Either anomaly 1, 3, 7, 8, 9 or 10 has occured"))
                self.time_object_went_missing = time.time()

            # if more than 10 seconds has gone by without the object arriving at the IR sensor then cast anomaly that shuts down the system
            elif time.time() - self.time_object_went_missing > 10:
                self.anomaly_logs.append((f"Conveyor {self.state.id}", "Mitigation for anomaly 1, 3, 7, 8, 9 or 10 has failed"))

        # if the virtual object has been placed on a conveyor then update the object
        elif placed_position == "Conveyor":
            opposite_id = int(not self.state.id)
            self.state = self.states[f"Conveyor_{opposite_id}"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0

        # if the virtual object has been placed in storage then update the object
        elif placed_position == "Storage":
            self.state = self.states[f"Storage_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0

        # if the virtual object has been picked up by a robot then update the object
        elif picked_up:
            print("Getting picked up")
            self.state = self.states[f"Robot_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0

        # Increment time if conveyor belt is running
        if conveyor_running:
            self.current_state_progress += self.step_size

    # MAYBE REFACTOR THIS TO MAKE THE LOGIC INSIDE THE ANIMATION CLASS WHERE IT BELONGS
    def get_info(self) -> tuple[ObjectState, float]:
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
