from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape


@dataclass
class CacheEntry:
    shape: str
    color: str
    progress: float
    status: str  # Values: Normal, Late, Early
    time_to_live: float


class ConveyorCache:
    def __init__(self, step_size: float):
        self.cache = []
        self.step_size = step_size
        self.entry_life_time = 10  # Seconds
        self.anomaly_threshold_time = 5  # seconds
        self.minimum_entry_count_to_cast_anomaly = 2  # entries
        self.progress_threshold = 1 / 3
        self.last_status = None
        self.status_time = 0

    def _valid_entry(self, entry: CacheEntry) -> bool:
        cached_entry = next((item for item in self.cache if entry.shape == item.shape and entry.color == item.color), None)

        # If this objects isn't already in the cache
        if cached_entry is None:
            return True

        # The object has moved at least the threshold distance since the last entry
        elif abs(entry.progress - cached_entry.progress) > self.progress_threshold:
            return True

        else:
            return False

    def _evaluate_conveyor_status(self) -> str:
        if len(self.cache) >= self.minimum_entry_count_to_cast_anomaly:
            status = self.cache[0].status
            for entry in self.cache:
                if entry.status != status:
                    return "Normal"

            return status

        return "Normal"

    def add_to_cache(self, shape: ObjectShape, color: ObjectColor, progress: float, status: str) -> None:
        entry = CacheEntry(shape, color, progress, status, self.entry_life_time)

        if self._valid_entry(entry):
            self.cache.append(entry)

    def step(self) -> bool:
        for entry in self.cache[:]:
            entry.time_to_live -= self.step_size

            if entry.time_to_live <= 0:
                self.cache.remove(entry)

        # Check if all the objects have the same status
        current_status = self._check_all_entry_status()
        if current_status != "Normal":
            if current_status == self.last_status:
                self.status_time += self.step_size
                if self.status_time > self.anomaly_threshold_time:
                    return True
                else:
                    return False
            else:
                self.last_status = current_status
                self.status_time = 0
                return False
        else:
            self.last_status = "Normal"
            self.status_time = 0
            return False
