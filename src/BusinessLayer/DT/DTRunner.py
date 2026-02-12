import src.BusinessLayer.DT.VirtualObject as VirtualObject
from src.BusinessLayer.DT.TimeBasedDT.TimeBasedDT import TimeBasedDT
from pyniryo import *
import time
import threading

class DTRunner():

    def __init__(self):
        self.stepSize = 0.2
        self.DTModel = TimeBasedDT(self.stepSize)
        self.simulateThread = None
        self.lock = threading.Lock()
        self.running = False

    def setRules(self, rules):
        self.DTModel.setRules(rules)

    def startDT(self):
        self.running = True
        self.simulateThread = threading.Thread(target=self.simulate)
        self.simulateThread.start()

    def createEvent(self, event):
        with self.lock:
            self.DTModel.createEvent(event)

    def stopDT(self):
        self.running = False
        self.simulateThread.join()

    def simulate(self):
        current_time = time.time()
        while (self.running):
            interval_start = time.time()
            with self.lock:
                self.DTModel.step()
            current_time = time.time()
            time.sleep(self.stepSize - (current_time - interval_start))
