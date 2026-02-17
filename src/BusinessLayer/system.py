from src.BusinessLayer.robot import *
from src.BusinessLayer.DT.DTRunner import DTRunner
from resources.environment import configuration
import keyboard

class System:
    def __init__(self, ips, positions):
        self.RobotArms = []
        self.running = True
        self.storageObjects = configuration["StorageObjects"]
        self.DT = DTRunner()
        self.lock = threading.Lock()

        # Add all the robot arms
        for i in range(len(ips)):
            ID = i
            IP_address = ips[i]
            poses = positions[i]
            self.RobotArms.append(RobotArm(IP_address, poses, ID))

    def stopSystem(self):
        print("STOP")
        self.running = False
        self.DT.running = False

    def Setup(self):
        self.DT.startDT()


        self.threads = []

        t_ir = threading.Thread(target=self.IR_Listener)
        self.threads.append(t_ir)
        t_ir.start()


        for arm in self.RobotArms:
            t = threading.Thread(target=self.robot_worker, args=(arm,))
            self.threads.append(t)
            t.start()

        t_anomaly = threading.Thread(target=self.anomalyListener)
        self.threads.append(t_anomaly)
        t_anomaly.start()

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
        with self.lock:
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

            rules = self.makeRuleFromEvent(object, destination)

            for i in range(len(self.RobotArms)):
                self.RobotArms[i].setRules(rules[i])

            self.DT.setRules(rules)

    def makeRuleFromEvent(self, object, destination):
        rules = [{}, {}]
        # Move to different storage
        storageID = int(object.position[-1])
        oppositeID = int(not storageID)
        if (object.position != destination):
            rules[oppositeID][(object.shape, object.color)] = "Storage"

        # Move to same storage
        else:
            rules[oppositeID][(object.shape, object.color)] = "Conveyor"
            rules[storageID][(object.shape, object.color)] = "Storage"

        return rules

    def anomalyListener(self):
        while (self.running):
            for robotArm in self.RobotArms:
                messages = robotArm.getAnomalyUpdates()
                for message in messages:
                    if (message[0] == "Stop System"):
                        self.stopSystem()
                        print("Human Intervention Required")
                    elif (message[0] == "Anomaly 5"):
                        id, shape, color = message[1]
                        self.anomaly5Mitigation(id, shape, color)

            time.sleep(0.1)

    def anomaly5Mitigation(self, robotIDArrival, shape, color):
        print(f"ID: {robotIDArrival}")
        goalStorageID = None
        for storageObject in self.storageObjects:
            if (storageObject.shape == shape and storageObject.color == color):
                if storageObject.position == "In_Transit":
                    for robotId in range(len(self.RobotArms)):
                        if (self.RobotArms[robotId].getRules().get((shape, color)) == "Storage"):
                            goalStorageID = robotId
                else:
                    print("happend")
                    goalStorageID = int(storageObject.position[-1])
                    print(goalStorageID)
                break
        if (goalStorageID == robotIDArrival):
            print("TRUE")
            self.RobotArms[robotIDArrival].setRules({(shape, color): "Storage"})
            self.RobotArms[robotIDArrival].addToQueue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
            self.RobotArms[robotIDArrival].setMitigationMode(False)
        else:
            print("False")
            self.RobotArms[robotIDArrival].setRules({(shape, color): "Conveyor"})
            self.RobotArms[int(not robotIDArrival)].setRules({(shape, color): "Storage"})
            self.RobotArms[robotIDArrival].addToQueue(configuration["PickFromIRSensorPriority"], "Conveyor", shape, color)
            self.RobotArms[robotIDArrival].setMitigationMode(False)


    def IR_Listener(self):
        has_received_false = [False, False]
        while(self.running):
            for ID in range(len(self.RobotArms)):
                robotArm = self.RobotArms[ID]
                if (robotArm.getIR()):
                    if (has_received_false[ID]):
                        with self.lock:
                            self.DT.createEvent(("IR", ID))
                        has_received_false[ID] = False
                else:
                    has_received_false[ID] = True

            time.sleep(0.1)





