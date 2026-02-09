import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from PresentationLayer.GUI import *
from resources.environment import configuration
from src.BusinessLayer.system import System
import threading

if __name__ == "__main__":
    IPs = configuration["ips"]
    positions = configuration["positions"]
    system = System(IPs, positions)
    t = threading.Thread(target=system.Setup, daemon=True)
    t.start()
    gui = GUI(system)
