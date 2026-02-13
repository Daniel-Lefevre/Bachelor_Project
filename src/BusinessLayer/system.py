from src.BusinessLayer.robot import *
from src.BusinessLayer.DT.DTRunner import DTRunner
from src.BusinessLayer.Rules import Rules
from resources.environment import StorageObject, configuration

class System:
    def __init__(self, ips, positions):
        self.RobotArms = []
        self.running = True
        self.storageObjects = configuration["StorageObjects"]
        self.Rules = Rules()
        self.DT = DTRunner()
        self.DT.setRules(self.Rules.getRules())
        self.lock = threading.Lock()

        # Add all the robot arms
        for i in range(len(ips)):
            ID = i
            IP_address = ips[i]
            poses = positions[i]
            self.RobotArms.append(RobotArm(IP_address, poses, ID))

    def stopSystem(self):
        self.running = False
        self.DT.running = False

    def Setup(self):
        self.DT.startDT()

        t_ir = threading.Thread(target=self.IR_Listener, daemon=True)
        t_ir.start()

        self.threads = []
        for arm in self.RobotArms:
            t = threading.Thread(target=self.robot_worker, args=(arm,), daemon=True)
            self.threads.append(t)
            t.start()

        self.startupRobotsLoop()

    def robot_worker(self, arm):
        # Setup Phase
        print(f"Robot {arm.ID} is initializing")
        arm.setUp()
        print(f"Robot {arm.ID} setup finished")

        # 2. Monitoring Phase
        while self.running:
            arm.Loop()
            time.sleep(0.05)

        # Disconnect when not running
        arm.disconnect()
        print("Everything has been shut down")

    def startupRobotsLoop(self):
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
        with self.lock:
            object = self.findObjectByName(name)

            self.DT.createEvent(("Pick Up", object))

            # Tell to pick up from storage
            self.RobotArms[int(object.position[-1])].addToQueue(configuration["PickFromStoragePriority"], "Storage", object.shape, object.color)

            self.Rules.makeRuleFromEvent(object, destination)
            rules = self.Rules.getRules()

            for i in range(len(self.RobotArms)):
                self.RobotArms[i].setRules(rules[i])

            self.DT.setRules(rules)

    def IR_Listener(self):
        has_received_false = [False, False]
        while(self.running):
            for ID in range(len(self.RobotArms)):
                robotArm = self.RobotArms[ID]
                if (robotArm.IR):
                    if (has_received_false[ID]):
                        with self.lock:
                            self.DT.createEvent(("IR", ID))
                        has_received_false[ID] = False
                else:
                    has_received_false[ID] = True

            time.sleep(0.1)





