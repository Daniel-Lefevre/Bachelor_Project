from src.BusinessLayer.robot import *
from src.BusinessLayer.DT.DTRunner import DTRunner
from src.BusinessLayer.Rules import Rules
from resources.environment import StorageObject

class System:
    def __init__(self, ips, positions):
        self.RobotArms = []
        self.running = True
        self.storageObjects = [
            StorageObject("Red Square", ObjectShape.SQUARE, ObjectColor.RED, "Storage 1"),
            StorageObject("Blue Square", ObjectShape.SQUARE, ObjectColor.BLUE, "Storage 0"),
            StorageObject("Green Square", ObjectShape.SQUARE, ObjectColor.GREEN, "Storage 1"),
            StorageObject("Red Circle", ObjectShape.CIRCLE, ObjectColor.RED, "Storage 0"),
            StorageObject("Blue Circle", ObjectShape.CIRCLE, ObjectColor.BLUE, "Storage 1"),
            StorageObject("Green Circle", ObjectShape.CIRCLE, ObjectColor.GREEN, "Storage 0"),
        ]
        self.Rules = Rules()
        self.DT = DTRunner()
        self.DT.setRules(self.Rules.getRules("DT"))

        # Add all the robot arms
        for i in range(len(ips)):
            ID = i
            IP_address = ips[i]
            poses = positions[i]
            self.RobotArms.append(RobotArm(IP_address, poses, ID))

    def setRunning(self, condition):
        self.running = condition

    def robot_worker(self, arm):
        # Setup Phase
        print(f"Robot {arm.ID} is initializing")
        arm.setUp()
        print(f"Robot {arm.ID} setup finished")

        # 2. Monitoring Phase
        while self.running:
            arm.Loop()
            time.sleep(0.05)

    def Setup(self):
        self.DT.startDT(120)

        t = threading.Thread(target=self.startupRobots, daemon=True)
        t.start()

    def startupRobots(self):
        self.threads = []
        # Start one thread per robot arm
        for arm in self.RobotArms:
            t = threading.Thread(target=self.robot_worker, args=(arm,))
            t.daemon = True
            self.threads.append(t)
            t.start()

        # Check for events to do
        while self.running:
            # Robot 0 take image
            if keyboard.is_pressed('9'):
                self.RobotArms[0].takeImage()
                time.sleep(0.5)

            # Robot 1 take image
            elif keyboard.is_pressed('2'):
                self.RobotArms[1].takeImage()
                time.sleep(0.5)

            time.sleep(0.05)

        # Wait for the threads to finnish their task before shutting down
        for t in self.threads:
            t.join()

        # Disconnect the arms
        for arm in self.RobotArms:
            arm.disconnect()

        print("Everything has been shut down")

    def updateObject(self, shape, color, position):
        for i in range(len(self.storageObjects)):
            object = self.storageObjects[i]
            if (object.shape == shape and object.color == color):
                self.storageObjects[i].position = position

    def getObjects(self):
        # Retrieve updates from the robot arms
        for arm in self.RobotArms:
            for update in arm.getObjectUpdates():
                self.updateObject(*update)

        return self.storageObjects

    def findObjectByName(self, objectName):
        for object in self.storageObjects:
            if (object.name == objectName):
                return object

    def moveObject(self, name, destination):
        object = self.findObjectByName(name)

        print(object)
        self.DT.event(("Pick Up", object))

        # Tell to pick up from storage
        self.RobotArms[int(object.position[-1])].addToQueue(configuration["PickFromStoragePriority"], "Storage", object.shape, object.color)

        self.Rules.makeRuleFromEvent(object, destination)
        robotrules = self.Rules.getRules("Robots")

        for i in range(len(self.RobotArms)):
            self.RobotArms[i].setRules(robotrules[i])

        DTrules = self.Rules.getRules("DT")
        self.DT.setRules(DTrules)




