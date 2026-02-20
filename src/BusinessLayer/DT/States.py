from typing import NamedTuple, Optional

from resources.environment import configuration


class ObjectState(NamedTuple):
    key: str
    origin: str
    id: int
    destination: Optional[str] = None
    time: Optional[float] = None


ObjectStates = {
    "Storage_0": ObjectState(key="Storage_0", origin="Storage", id=0),
    "Storage_1": ObjectState(key="Storage_1", origin="Storage", id=1),
    "Conveyor_0": ObjectState(key="Conveyor_0", origin="Conveyor", id=0, time=configuration["Observed_times"]["Conveyor_0"]),
    "Conveyor_1": ObjectState(key="Conveyor_1", origin="Conveyor", id=1, time=configuration["Observed_times"]["Conveyor_1"]),
    "Robot_0_Conveyor_to_Standby": ObjectState(key="Robot_0_Conveyor_to_Standby", origin="Conveyor", destination="Standby", id=0),
    "Robot_1_Conveyor_to_Standby": ObjectState(key="Robot_1_Conveyor_to_Standby", origin="Conveyor", destination="Standby", id=1),
    "Robot_0_Storage_to_Standby": ObjectState(key="Robot_0_Storage_to_Standby", origin="Storage", destination="Standby", id=0),
    "Robot_1_Storage_to_Standby": ObjectState(key="Robot_1_Storage_to_Standby", origin="Storage", destination="Standby", id=1),
    "Robot_0_Standby": ObjectState(key="Standby_0", origin="Robot", destination="Robot", id=0),
    "Robot_1_Standby": ObjectState(key="Standby_1", origin="Robot", destination="Robot", id=1),
    "Robot_0_Standby_to_Conveyor": ObjectState(key="Robot_0_Standby_to_Conveyor", origin="Standby", destination="Conveyor", id=0),
    "Robot_1_Standby_to_Conveyor": ObjectState(key="Robot_1_Standby_to_Conveyor", origin="Standby", destination="Conveyor", id=1),
    "Robot_0_Conveyor_to_Storage": ObjectState(key="Robot_0_Conveyor_to_Storage", origin="Conveyor", destination="Storage", id=0),
    "Robot_1_Conveyor_to_Storage": ObjectState(key="Robot_1_Conveyor_to_Storage", origin="Conveyor", destination="Storage", id=1),
    "IR_0": ObjectState(key="IR_0", origin="IR", id=0),
    "IR_1": ObjectState(key="IR_1", origin="IR", id=1),
}


class RobotStates(NamedTuple):
    key: str
    origin: str
    time: float
    destination: Optional[str] = None


def create_robot_states(id: int) -> dict:
    return {
        "Observation": RobotStates(key="Observation", origin="Observation", time=float("inf")),
        "Standby": RobotStates(key="Standby", origin="Standby", time=float("inf")),
        "Observation_to_Storage": RobotStates(key="Observation_to_Storage", origin="Observation", destination="Storage", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Storage"]),
        "Observation_to_Conveyor": RobotStates(key="Observation_to_Conveyor", origin="Observation", destination="Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Conveyor"]),
        "Storage_to_Standby": RobotStates(key="Storage_to_Standby", origin="Storage", destination="Standby", time=configuration["Observed_times"][f"Robot_{id}_Storage_to_Standby"]),
        "Storage_to_Observation": RobotStates(key="Storage_to_Observation", origin="Storage", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Storage_to_Observation"]),
        "Conveyor_to_Standby": RobotStates(key="Conveyor_to_Standby", origin="Conveyor", destination="Standby", time=configuration["Observed_times"][f"Robot_{id}_Conveyor_to_Standby"]),
        "Standby_to_Conveyor": RobotStates(key="Safe_to_Conveyor", origin="Standby", destination="Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Standby_to_Conveyor"]),
        "Conveyor_to_Observation_pickup": RobotStates(key="Conveyor_to_Observation_pickup", origin="Conveyor", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Conveyor_to_Observation_pickup"]),
        "Conveyor_to_Observation_place": RobotStates(key="Conveyor_to_Observation_place", origin="Conveyor", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Conveyor_to_Observation_place"]),
    }
