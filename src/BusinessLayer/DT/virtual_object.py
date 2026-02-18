from __future__ import annotations

from typing import TYPE_CHECKING

from src.BusinessLayer.DT.states import ObjectStates

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape


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

    def step(self, pick_up_destination: str | None, placed_position: str | None, conveyor_running: bool, conveyor_id_to_be_left: int | None):
        if pick_up_destination:
            origin = "Conveyor" if self.state.origin == "IR" else "Storage"
            self.state = self.states[f"Robot_{self.state.id}_{origin}_to_{pick_up_destination}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0
        elif placed_position == "Conveyor":
            opposite_id = int(not self.state.id)
            self.state = self.states[f"Conveyor_{opposite_id}"]
            self.current_state_progress_goal = self.state.time
            self.current_state_progress = 0
        elif placed_position == "Storage":
            self.state = self.states[f"Storage_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0
        elif conveyor_id_to_be_left is not None and self.state.origin == "Conveyor" and self.state.id == conveyor_id_to_be_left:
            # Check that the object has arrived to early
            if self.current_state_progress_goal - self.current_state_progress > 1.0:
                print("BIG IMPORTANT ERROR: SOMETHING UNEXPECTED IN IR (ARRIVED TOO EARLY)")
            self.state = self.states[f"IR_{self.state.id}"]
            self.current_state_progress_goal = float("inf")
            self.current_state_progress = 0
            self.has_reached_ir = True
        elif self.current_state_progress_goal - self.current_state_progress < -1.0:
            print("BIG IMPORTANT ERROR: SOMETHING MISSING IN IR (ARRIVED TOO LATE)")

        # Increment time if conveyor belt is running
        if conveyor_running:
            self.current_state_progress += self.step_size
