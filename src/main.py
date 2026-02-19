import os
import sys
import threading

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from resources.environment import configuration
from src.BusinessLayer.system import System
from src.PresentationLayer.GUI import GUI

if __name__ == "__main__":
    IPs = configuration["ips"]
    positions = configuration["positions"]
    system = System(IPs, positions)
    t = threading.Thread(target=system.set_up, daemon=True)
    t.start()
    gui = GUI(system)
