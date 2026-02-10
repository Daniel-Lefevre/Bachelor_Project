from resources.environment import configuration

class Rules:
    def __init__(self):
        self.robotRules = [{}, {}]
        self.DTRules = {}
        storageObjects = configuration["StorageObjects"]
        for obj in storageObjects:
            self.DTRules[(obj.shape, obj.color)] = (None, None)

    def makeRuleFromEvent(self, object, destination):
        # Move to different storage
        storageID = int(object.position[-1])
        oppositeID = int(not storageID)
        if (object.position != destination):
            self.robotRules[oppositeID][(object.shape, object.color)] = "Storage"
            nextState = f"Robot_{oppositeID}_Conveyor_to_Storage"
            self.DTRules[(object.shape, object.color)] = (nextState, None) if storageID else (None, nextState)

        # Move to same storage
        else:
            self.robotRules[oppositeID][(object.shape, object.color)] =  "Conveyor"
            self.robotRules[storageID][(object.shape, object.color)] = "Storage"

            homeState = f"Robot_{storageID}_Conveyor_to_Storage"
            awayState = f"Robot_{oppositeID}_Conveyor_to_Conveyor"
            self.DTRules[(object.shape, object.color)] = (awayState, homeState) if storageID else (homeState, awayState)

    def getRules(self, name):
        if name == "Robots":
            return self.robotRules
        elif name == "DT":
            return self.DTRules
        else:
            print("No such rules to return")
            return None
