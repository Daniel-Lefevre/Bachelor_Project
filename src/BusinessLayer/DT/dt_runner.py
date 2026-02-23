from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from src.BusinessLayer.DT.TimeBasedDT.time_based_dt import TimeBasedDT

if TYPE_CHECKING:
    from resources.environment import StorageObject


class DTRunner:
    def __init__(self):
        self.step_size = 0.2
        self.dt_model = TimeBasedDT(self.step_size)
        self.simulate_thread = None
        self.lock = threading.Lock()
        self.running = False

    def set_rules(self, rules: list[dict]) -> None:
        self.dt_model.set_rules(rules)

    def start_dt(self) -> None:
        self.running = True
        self.simulate_thread = threading.Thread(target=self._simulate)
        self.simulate_thread.start()

    def create_event(self, event: tuple[str, int | StorageObject | None]) -> None:
        with self.lock:
            self.dt_model.create_event(event)

    def stop_dt(self) -> None:
        self.running = False
        self.simulate_thread.join()

    def _simulate(self) -> None:
        while self.running:
            interval_start = time.time()
            with self.lock:
                self.dt_model.step()
            current_time = time.time()
            time.sleep(self.step_size - (current_time - interval_start))

    def get_info_dt(self) -> dict[list, list, list]:
        return self.dt_model.get_info_dt()
