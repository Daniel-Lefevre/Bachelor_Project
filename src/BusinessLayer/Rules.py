from resources.environment import configuration

class Rules:
    def __init__(self):
        self.rules = [{}, {}]

    def makeRuleFromEvent(self, object, destination):
        # Move to different storage
        storageID = int(object.position[-1])
        oppositeID = int(not storageID)
        if (object.position != destination):
            self.rules[oppositeID][(object.shape, object.color)] = "Storage"

        # Move to same storage
        else:
            self.rules[oppositeID][(object.shape, object.color)] =  "Conveyor"
            self.rules[storageID][(object.shape, object.color)] = "Storage"

    def getRules(self):
        return self.rules
