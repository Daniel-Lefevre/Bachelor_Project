# Bachelor_Project

### **Project Report**
https://www.overleaf.com/9756798451dxcbzdtvnrzf#d03e32

### **Gantt Diagram**
https://docs.google.com/spreadsheets/d/1Zpuhi13NQvuIfRxC9UdOiqnWY_iOfHch5isNjrXS12Y/edit?usp=sharing

###  **Project Notes**
https://docs.google.com/document/d/1T-WWYX9KqtfHGER0PO_sFUBXoYqzswZ6ThZjRrAKvsA/edit?usp=sharing

### **Drawings**
https://drive.google.com/file/d/1xx9qzL80M35pHLFEnVWRHDTKTx0Pse91/view?usp=sharing

### **Anomalies**
https://docs.google.com/spreadsheets/d/1-2GoGnaEigZ-x1MziMF2Ia3y4VUbnZ09kIiUq5JIvcc/edit?usp=sharing

### **Results**
https://docs.google.com/spreadsheets/d/1Awrg3cTaBOSe2nA5g6LJq93RP5s03s4xuAfEhwDZIqU/edit?usp=sharing


### **How to Connect To The Robots**
To connect to the robots setup the following environment

<img width="460" height="269" alt="image" src="https://github.com/user-attachments/assets/aa01015e-e0c4-42f3-b40d-f9254778088d" />

When the environment has been setup and the cable from the switch has been connected to the laptop. Go to `control panel` $\to$ `network and internet` $\to$ `Network and Sharing Center` $\to$ `Change adapter settings`. Right click the enternet connection that has been setup with the cable and move into `IPv4` $\to$ `Properties`. Select `Use the following IP-address` and enter:

- **IP-address**: 169.254.200.100
- **Subnet mask**: 255.255.0.0

Click OK and apply. You should not be able to connect to the robots. This can be checked via opening a command promt and typing: `SSH niryo@IP-address`, where the IP-address is either of the two IP-addresses of the robots. The password for both robots are `robotics`.


### **Problems Encountered**
During the development, it was encoutered that the built in `pick_from_pose` method in the Pyniryo library was observed to be inconsisten. The robot would after a random amount of time begin to miss objects when calling the function. To fix this a manual offset was implemented. This was done via. the method: `_correct_robot_offset`. If this problem keeps occuring, adjust the values with the right hand rule.

### **Switch System**
To switch between the baseline, time-based DT and the vision-based DT, three branches has been created.


| System          | Branch                 |
|-----------------|------------------------|
| Baseline        | origin/Baseline-System |
| time-based DT   | origin/Time-Based-DT   |
| vision-based DT | origin/Development     |

*The main branch is not used.*

