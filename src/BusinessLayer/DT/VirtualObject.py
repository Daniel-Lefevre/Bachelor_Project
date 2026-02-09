from enum import Enum
from resources.environment import configuration

class ObjectState(str, Enum):
    STORAGE_0 = "Storage_0"
    STORAGE_1 = "Storage_1"
    CONVEYOR_0 = "Conveyor_0"
    CONVEYOR_1 = "Conveyor_1"
    ROBOT_0_STORAGE_TO_CONVEYOR = "Robot_0_Storage_to_Conveyor"
    ROBOT_1_STORAGE_TO_CONVEYOR = "Robot_1_Storage_to_Conveyor"
    ROBOT_0_CONVEYOR_TO_CONVEYOR = "Robot_0_Conveyor_to_Conveyor"
    ROBOT_1_CONVEYOR_TO_CONVEYOR = "Robot_1_Conveyor_to_Conveyor"
    IR_0 = "IR_0"
    IR_1 = "IR_1"
    ROBOT_0_CONVEYOR_TO_STORAGE = "Robot_0_Conveyor_to_Storage"
    ROBOT_1_CONVEYOR_TO_STORAGE = "Robot_1_Conveyor_to_Storage"

class VirtualObject:
    def __init__(self, shape, color, state: ObjectState, stepSize):
        self.state = state
        self.color = color
        self.shape = shape
        self.innerClock = 0
        self.conveyorPosition = 0
        self.timeSinceStateTransition = 0
        self.stepSize = stepSize

    def pickUpObject(self):
        if self.state == "Storage_0":
            self.state = "Robot_0_Storage_to_Conveyor"
        elif self.state == "Storage_1":
            self.state = "Robot_1_Storage_to_Conveyor"
        self.timeSinceStateTransition = 0

    def IncrementInnerClock(self):
        self.innerClock += self.stepSize
        self.timeSinceStateTransition += self.stepSize
        if (self.state != "Storage_0"):
            # We need to transition to new state
            if self.timeSinceStateTransition > configuration["Observed_times"][self.state]:
                if (self.state == "Conveyor_0"):
                    self.state = "IR_0"
                elif (self.state == "Conveyor_1"):
                    self.state = "IR_1"
                elif (self.state == "Robot_0_Storage_to_Conveyor"):
                    self.state = "Conveyor_1"
                elif (self.state == "Robot_1_Storage_to_Conveyor"):
                    self.state = "Conveyor_0"
                elif (self.state == "Robot_0_Conveyor_to_Conveyor"):
                    self.state = "Conveyor_1"
                elif (self.state == "Robot_1_Conveyor_to_Conveyor"):
                    self.state = "Conveyor_0"
                elif (self.state == "IR_0"):
                    self.state = "Robot_0_Conveyor_to_Storage"
                elif (self.state == "IR_1"):
                    self.state = "Robot_1_Conveyor_to_Conveyor"
                elif (self.state == "Robot_0_Conveyor_to_Storage"):
                    self.state = "Storage_0"
                print("State: " + str(self.state))
                self.timeSinceStateTransition = 0
