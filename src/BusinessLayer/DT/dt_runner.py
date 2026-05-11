from __future__ import annotations

import queue
import threading
import time
from datetime import datetime
from typing import TYPE_CHECKING

from src.BusinessLayer.DT.States import ObjectStates
from src.BusinessLayer.DT.VisionBasedDT.conveyor_vision_module import ConveyorVisionModule
from src.BusinessLayer.DT.VisionBasedDT.storage_vision_module import StorageVisionModule

# from src.BusinessLayer.DT.TimeBasedDT.time_based_dt import TimeBasedDT
from src.BusinessLayer.DT.VisionBasedDT.vision_based_dt import VisionBasedDT

if TYPE_CHECKING:
    import numpy as np

    from resources.environment import StorageObject


class DTRunner:
    def __init__(self):
        self.step_size = 0.2
        # self.dt_model = VisionBasedDT(self.step_size)
        self.dt_model = VisionBasedDT(self.step_size)
        self.simulate_thread = None
        self.dt_lock = threading.Lock()
        self.running = False
        self.image_queue = queue.Queue()
        self.image_worker_thread = None
        self.conveyor_vision_module = ConveyorVisionModule()
        self.storage_vision_module = StorageVisionModule()
        self.latest_ir_readings = (False, False)
        self.unknow_object = None
        self.has_logged_unknow = False

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

    def _image_worker(self):
        while self.running:
            try:
                image, metadata, state_snapshot = self.image_queue.get(timeout=1)
                robot_id = metadata.id
                location = metadata.location
                unknown_object = None
                return_objects = []

                if location == "Conveyors":
                    # Heavy processing
                    unknown_object, return_objects = self.conveyor_vision_module.compare_image_with_DT(image, robot_id, state_snapshot)

                elif location == "Storage":
                    unknown_object, return_objects = self.storage_vision_module.compare_image_with_DT(image, robot_id, state_snapshot)

                self._update_state_snapshots(return_objects, location)

                if unknown_object is not None:
                    self.unknow_object = unknown_object

                event = ("Update_DT", return_objects)

                # Update DT INSIDE lock
                with self.dt_lock:
                    self.dt_model.create_event(event)

            except queue.Empty:
                continue

    def _update_state_snapshots(self, return_objects: list, location: str) -> None:
        temp_list = []

        while not self.image_queue.empty():
            image, metadata, state_snapshot = self.image_queue.get()
            robot_id = metadata.id
            location = metadata.location

            for virtual_object in state_snapshot.objects:
                for return_object in return_objects:
                    if virtual_object.color == return_object.color and virtual_object.shape == return_object.shape:
                        # Correct all updates in dictionary from the vision module to the DT

                        if location == "Conveyor":
                            for update_category in return_object.updates:
                                if update_category == "error_correction":
                                    virtual_object.progress += return_object[update_category]

                        elif location == "Storage":
                            for update_category in return_object.updates:
                                if update_category == "Updated_State":
                                    origin = {return_object[update_category].origin}
                                    storage_id = {return_object[update_category].id}
                                    state_key = f"{origin}_{storage_id}"

                                    virtual_object.state = ObjectStates[state_key]

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
            image, metadata = event_param
            event_param = (image, metadata, self.dt_model.get_state_snapshot())
            self.image_queue.put(event_param)
            return

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

        if self.unknow_object is not None and not self.has_logged_unknow:
            current_time = datetime.now().strftime("%H:%M")
            if self.unknow_object["Location"] == "Conveyors":
                info[1].append((current_time, f"Conveyor {self.unknow_object['Conveyor']}", "Anomaly 14"))
            elif self.unknow_object["Location"] == "Storage":
                info[1].append((current_time, f"Storage {self.unknow_object['Storage']}", "Anomaly 15"))
            self.has_logged_unknow = True

        # print(info[1])

        return info
