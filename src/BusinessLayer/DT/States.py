from typing import NamedTuple, Optional

from resources.environment import configuration


class ObjectState(NamedTuple):
    key: str
    origin: str
    id: int
    destination: Optional[str] = None
    time: Optional[float] = float("inf")


ObjectStates = {
    "Storage_0": ObjectState(key="Storage_0", origin="Storage", id=0, time=float("inf")),
    "Storage_1": ObjectState(key="Storage_1", origin="Storage", id=1, time=float("inf")),
    "Conveyor_0": ObjectState(key="Conveyor_0", origin="Conveyor", id=0, time=configuration["Observed_times"]["Conveyor_0"]),
    "Conveyor_1": ObjectState(key="Conveyor_1", origin="Conveyor", id=1, time=configuration["Observed_times"]["Conveyor_1"]),
    "IR_0": ObjectState(key="IR_0", origin="IR", id=0, time=float("inf")),
    "IR_1": ObjectState(key="IR_1", origin="IR", id=1, time=float("inf")),
    "Robot_0": ObjectState(key="Robot_0", origin="Robot", id=0, time=float("inf")),
    "Robot_1": ObjectState(key="Robot_1", origin="Robot", id=1, time=float("inf")),
}


class RobotStates(NamedTuple):
    key: str
    origin: str
    time: float
    destination: Optional[str] = None


def create_robot_states(id: int) -> dict:
    return {
        "Setup": RobotStates(key="Setup", origin="Setup", time=float("inf")),
        "Storage_Observation": RobotStates(key="Storage_Observation", origin="Storage_Observation", time=float("inf")),
        "Observation": RobotStates(key="Observation", origin="Observation", time=float("inf"), destination="Observation"),
        "Observation_to_Storage_Observation": RobotStates(key="Observation_to_Storage_Observation", origin="Observation", destination="Storage_Observation", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Storage_Observation"]),
        "Storage_Observation_to_Storage": RobotStates(key="Storage_Observation_to_Storage", origin="Storage_Observation", destination="Storage", time=configuration["Observed_times"][f"Robot_{id}_Storage_Observation_to_Storage"]),
        "Storage_to_Storage_Observation": RobotStates(key="Storage_to_Storage_Observation", origin="Storage", destination="Storage_Observation", time=configuration["Observed_times"][f"Robot_{id}_Storage_to_Storage_Observation"]),
        "Storage_Observation_to_Standby": RobotStates(key="Storage_Observation_to_Standby", origin="Storage_Observation", destination="Standby", time=configuration["Observed_times"][f"Robot_{id}_Storage_Observation_to_Standby"]),
        "Observation_to_Workspace_Observation": RobotStates(
            key="Observation_to_Workspace_Observation", origin="Observation", destination="Workspace_Observation", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Workspace_Observation"]
        ),
        "Workspace_Observation_to_Pickup_Conveyor": RobotStates(
            key="Workspace_Observation_to_Pickup_Conveyor", origin="Workspace_Observation", destination="Pickup_Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Workspace_Observation_to_Pickup_Conveyor"]
        ),
        "Pickup_Conveyor_to_Observation": RobotStates(key="Pickup_Conveyor_to_Observation", origin="Pickup_Conveyor", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Pickup_Conveyor_to_Observation"]),
        "Observation_to_Standby": RobotStates(key="Observation_to_Standby", origin="Observation", destination="Standby", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Standby"]),
        "Observation_to_Place_Storage": RobotStates(key="Observation_to_Place_Storage", origin="Observation", destination="Place_Storage", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Place_Storage"]),
        "Standby_to_Place_Conveyor": RobotStates(key="Standby_to_Place_Conveyor", origin="Standby", destination="Place_Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Standby_to_Place_Conveyor"]),
        "Standby": RobotStates(key="Standby", origin="Standby", destination="Standby", time=float("inf")),
        "Place_Storage_to_Observation": RobotStates(key="Place_Storage_to_Observation", origin="Place_Storage", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Place_Storage_to_Observation"]),
        "Place_Conveyor_to_Observation": RobotStates(key="Place_Conveyor_to_Observation", origin="Place_Conveyor", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Place_Conveyor_to_Observation"]),
    }
