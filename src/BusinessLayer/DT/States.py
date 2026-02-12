from typing import Literal, Optional, NamedTuple
from resources.environment import configuration

class ObjectState(NamedTuple):
    key: str
    origin: str
    id: int
    destination: Optional[str] = None
    time: Optional[float] = None

ObjectStates = {
    "Storage_0" : ObjectState(key="Storage_0", origin="Storage", id=0),
    "Storage_1" : ObjectState(key="Storage_1", origin="Storage", id=1),
    "Conveyor_0" : ObjectState(key="Conveyor_0", origin="Conveyor", id=0, time=configuration["Observed_times"]["Conveyor_0"]),
    "Conveyor_1" : ObjectState(key="Conveyor_1", origin="Conveyor", id=1, time=configuration["Observed_times"]["Conveyor_1"]),
    "Robot_0_Storage_to_Conveyor" : ObjectState(key="Robot_0_Storage_to_Conveyor", origin="Storage", id=0, destination="Conveyor"),
    "Robot_1_Storage_to_Conveyor" : ObjectState(key="Robot_1_Storage_to_Conveyor", origin="Storage", id=1, destination="Conveyor"),
    "Robot_0_Conveyor_to_Conveyor" : ObjectState(key="Robot_0_Conveyor_to_Conveyor", origin="Conveyor", destination="Conveyor", id=0),
    "Robot_1_Conveyor_to_Conveyor" : ObjectState(key="Robot_1_Conveyor_to_Conveyor", origin="Conveyor", destination="Conveyor", id=1),
    "Robot_0_Conveyor_to_Storage" : ObjectState(key="Robot_0_Conveyor_to_Storage", origin="Conveyor", destination="Storage", id=0),
    "Robot_1_Conveyor_to_Storage" : ObjectState(key="Robot_1_Conveyor_to_Storage", origin="Conveyor", destination="Storage", id=1),
    "IR_0" : ObjectState(key="IR_0", origin="IR", id=0),
    "IR_1" : ObjectState(key="IR_1", origin="IR", id=1)
}

class RobotStates(NamedTuple):
    key: str
    origin: str
    time: float
    destination: Optional[str] = None

def createRobotStates(id):
    return {
        "Observation" : RobotStates(key="Observation", origin="Observation", time=float('inf')),
        "Observation_to_Storage" : RobotStates(key="Observation_to_Storage", origin="Observation", destination="Storage", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Storage"]),
        "Observation_to_Conveyor" : RobotStates(key="Observation_to_Conveyor", origin="Observation", destination="Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Observation_to_Conveyor"]),
        "Storage_to_Conveyor" : RobotStates(key="Storage_to_Conveyor", origin="Storage", destination="Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Storage_to_Conveyor"]),
        "Storage_to_Observation" : RobotStates(key="Storage_to_Observation", origin="Storage", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Storage_to_Observation"]),
        "Conveyor_to_Conveyor" : RobotStates(key="Conveyor_to_Conveyor", origin="Conveyor", destination="Conveyor", time=configuration["Observed_times"][f"Robot_{id}_Conveyor_to_Conveyor"]),
        "Conveyor_to_Storage": RobotStates(key="Conveyor_to_Storage", origin="Conveyor", destination="Storage", time=configuration["Observed_times"][f"Robot_{id}_Conveyor_to_Storage"]),
        "Conveyor_to_Observation" : RobotStates(key="Conveyor_to_Observation", origin="Conveyor", destination="Observation", time=configuration["Observed_times"][f"Robot_{id}_Conveyor_to_Observation"])
    }
