from __future__ import annotations

import queue
import threading
import time
from typing import TYPE_CHECKING

from src.BusinessLayer.DT.TimeBasedDT.time_based_dt import TimeBasedDT

if TYPE_CHECKING:
    import numpy as np

    from resources.environment import StorageObject


class DTRunner:
    def __init__(self):
        self.step_size = 0.2
        self.dt_model = TimeBasedDT(self.step_size)
        self.simulate_thread = None
        self.dt_lock = threading.Lock()
        self.running = False
        self.image_queue = queue.Queue()
        self.image_worker_thread = None
        self.latest_ir_readings = (False, False)
        self.unknow_object = None
        self.has_logged_unknow = False
        self.storage_pickup_confirmation = "Waiting"
        self.storage_pickup_confirmation_id = None
        self.anomalies = []

    # Private Functions

    def _simulate(self) -> None:
        while self.running:
            interval_start = time.time()
            with self.dt_lock:
                self.dt_model.step(self.latest_ir_readings)
            current_time = time.time()
            sleep_time = self.step_size - (current_time - interval_start)
            if sleep_time < 0:
                # print(sleep_time)
                sleep_time = 0
            time.sleep(sleep_time)

    # Public Functions

    def set_rules(self, rules: list[dict]) -> None:
        with self.dt_lock:
            self.dt_model.set_rules(rules)

    def start_dt(self) -> None:
        self.running = True
        self.simulate_thread = threading.Thread(target=self._simulate)
        self.simulate_thread.start()

    def create_event(self, event: tuple[str | np.ndarray, int | StorageObject | None]) -> None:
        eventype, event_param = event

        if eventype == "IR":
            self.latest_ir_readings = event_param
            return

        with self.dt_lock:
            self.dt_model.create_event(event)

    def stop_dt(self) -> None:
        self.running = False
        self.simulate_thread.join()

        if self.image_worker_thread is not None:
            self.image_worker_thread.join()

    def get_info_dt(self) -> tuple[list[tuple[str, int, str]], dict[list, list, list]]:
        info = self.dt_model.get_info_dt()
        return info
