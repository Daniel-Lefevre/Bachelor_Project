from __future__ import annotations

import queue
import threading
import time
from typing import TYPE_CHECKING

# from src.BusinessLayer.DT.TimeBasedDT.time_based_dt import TimeBasedDT
from src.BusinessLayer.DT.VisionBasedDT.vision_based_dt import VisionBasedDT
from src.BusinessLayer.DT.VisionBasedDT.vision_module import VisionModule

if TYPE_CHECKING:
    import numpy as np

    from resources.environment import StorageObject


class DTRunner:
    def __init__(self):
        self.step_size = 0.4
        self.dt_model = VisionBasedDT(self.step_size)
        self.simulate_thread = None
        self.dt_lock = threading.Lock()
        self.running = False
        self.image_queue = queue.Queue()
        self.image_worker_thread = None
        self.vision_module = VisionModule()

    # Private Functions

    def _simulate(self) -> None:
        while self.running:
            interval_start = time.time()
            with self.dt_lock:
                self.dt_model.step()
            current_time = time.time()
            sleep_time = self.step_size - (current_time - interval_start)
            if sleep_time < 0:
                print(sleep_time)
                sleep_time = 0
            time.sleep(sleep_time)

    def _image_worker(self):
        while self.running:
            try:
                image, robot_id, state_snapshot = self.image_queue.get(timeout=1)

                # Heavy processing
                return_objects, conveyor_cache_entries = self.vision_module.compare_image_with_DT(image, robot_id, state_snapshot)
                event = ("Update_DT", (return_objects, conveyor_cache_entries))

                self._update_state_snapshots(return_objects)

                # Update DT INSIDE lock
                with self.dt_lock:
                    self.dt_model.create_event(event)

            except queue.Empty:
                continue

    def _update_state_snapshots(self, return_objects: list) -> None:
        temp_list = []

        while not self.image_queue.empty():
            image, robot_id, state_snapshot = self.image_queue.get()

            for virtual_object in state_snapshot.objects:
                for return_object in return_objects:
                    if virtual_object.color == return_object.color and virtual_object.shape == return_object.shape:
                        virtual_object.progress += return_object.error_correction

            temp_list.append((image, robot_id, state_snapshot))

        for item in temp_list:
            self.image_queue.put(item)

    # Public Functions

    def set_rules(self, rules: list[dict]) -> None:
        with self.dt_lock:
            self.dt_model.set_rules(rules)

    def start_dt(self) -> None:
        self.running = True
        self.simulate_thread = threading.Thread(target=self._simulate)
        self.simulate_thread.start()

        self.image_worker_thread = threading.Thread(target=self._image_worker)
        self.image_worker_thread.start()

    def create_event(self, event: tuple[str | np.ndarray, int | StorageObject | None]) -> None:
        eventype, event_param = event

        if eventype == "Image":
            image, robot_id = event_param
            event_param = (image, robot_id, self.dt_model.get_state_snapshot())
            self.image_queue.put(event_param)
            return

        with self.dt_lock:
            self.dt_model.create_event(event)

    def stop_dt(self) -> None:
        self.running = False
        self.simulate_thread.join()

        if self.image_worker_thread is not None:
            self.image_worker_thread.join()

    def get_info_dt(self) -> tuple[list[tuple[str, int, str]], dict[list, list, list]]:
        return self.dt_model.get_info_dt()
