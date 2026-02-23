from __future__ import annotations

from typing import TYPE_CHECKING

from src.BusinessLayer.DT.states import ObjectStates

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape

    from src.BusinessLayer.DT.states import ObjectState


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

    def set_ir_state(self, state: bool) -> None:
        self.has_reached_ir = state

    def get_ir_state(self) -> bool:
        return self.has_reached_ir

    def get_state(self):
        return self.state

    def is_at_drop_off(self, conveyor_id: int) -> bool:
        if self.state.key == f"Conveyor_{conveyor_id}":
            return self.current_state_progress / self.current_state_progress_goal < 1 / 5
        else:
            return False

    def step(self, picked_up: bool, placed_position: str | None, conveyor_running: bool, activated_ir_id: int | None):
        if activated_ir_id is not None and self.state.origin == "Conveyor" and self.state.id == activated_ir_id:
            # Check that the object has arrived to early
            if self.current_state_progress_goal - self.current_state_progress > 1.0:
                print("Either anomaly 2 or 7 has occured")
            elif self.current_state_progress_goal - self.current_state_progress < -1.0:
                print("Either anomaly 1 or 7 has occured")

            self.state = self.states[f"IR_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0
            self.has_reached_ir = True

        elif self.current_state_progress_goal - self.current_state_progress < -1.0:
            print("Either anomaly 1, 3, 7, 8, 9 or 10 has occured")

        elif placed_position == "Conveyor":
            opposite_id = int(not self.state.id)
            self.state = self.states[f"Conveyor_{opposite_id}"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0

        elif placed_position == "Storage":
            self.state = self.states[f"Storage_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0

        elif picked_up:
            self.state = self.states[f"Robot_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0

        # Increment time if conveyor belt is running
        if conveyor_running:
            self.current_state_progress += self.step_size

    def get_info(self) -> tuple[ObjectState, float]:
        progress = 0
        if (self.current_state_progress_goal == float("inf")):
            progress = float("inf")
        elif (self.current_state_progress > self.current_state_progress_goal):
            progress = 1
        else:
            progress = self.current_state_progress / self.current_state_progress_goal
        return (self.state, progress)
