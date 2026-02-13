from pyniryo import ObjectShape, ObjectColor
from dataclasses import dataclass
from typing import Literal

@dataclass
class StorageObject:
    name: str
    shape: Literal[ObjectShape.CIRCLE, ObjectShape.SQUARE]
    color: Literal[ObjectColor.RED, ObjectColor.BLUE, ObjectColor.GREEN]
    position: Literal["Storage_1", "In_Transit", "Storage_0"]


configuration = {
    # IP's of the niryo ned2 robots
    "ips": ["169.254.200.200", "169.254.200.201"],

    # Order of execution for the robot arms
    "NumberOfPriorities" : 2,
    "PickFromStoragePriority" : 2,
    "PickFromIRSensorPriority" : 1,

    # Overview of equipment in the experimental setup
    "NumberOfRobotArms": 2,
    "NumberOfConveyors": 2,

    # Starting setup of the system
    "StorageObjects" : [
        StorageObject("Red Square", ObjectShape.SQUARE, ObjectColor.RED, "Storage_1"),
        StorageObject("Blue Square", ObjectShape.SQUARE, ObjectColor.BLUE, "Storage_0"),
        StorageObject("Green Square", ObjectShape.SQUARE, ObjectColor.GREEN, "Storage_1"),
        StorageObject("Red Circle", ObjectShape.CIRCLE, ObjectColor.RED, "Storage_0"),
        StorageObject("Blue Circle", ObjectShape.CIRCLE, ObjectColor.BLUE, "Storage_1"),
        StorageObject("Green Circle", ObjectShape.CIRCLE, ObjectColor.GREEN, "Storage_0"),
    ],

    # Camera configurations found using experiments
    "brightness" : [[1.24816061, 1.32431245], [1.28829838, 1.42241655]],
    "contrast" : [[1.01757065, 1.02363494], [1.02113107, 1.01575003]],
    "saturation" : [[1.23197705, 1.35534524], [1.27477756, 1.22645163]],

    # Times for the DT
    "Observed_times" : {
        "Conveyor_0" : 9.15,
        "Conveyor_1" : 8.76,
        "Robot_0_Conveyor_to_Conveyor" : 6.07,
        "Robot_1_Conveyor_to_Conveyor" : 6.0,
        "Robot_0_Observation_to_Conveyor" : 4.35,
        "Robot_1_Observation_to_Conveyor" : 4.6,
        "Robot_0_Storage_to_Conveyor" : 7.01,
        "Robot_1_Storage_to_Conveyor" : 7.27,
        "Robot_0_Conveyor_to_Storage" : 7.7,
        "Robot_1_Conveyor_to_Storage" : 7.1,
        "Robot_0_Observation_to_Storage" : 7.93,
        "Robot_1_Observation_to_Storage" : 8.97,
        "Robot_0_Conveyor_to_Observation" : 3.03,
        "Robot_0_Storage_to_Observation" : 5.07,
        "Robot_1_Conveyor_to_Observation" : 4.4,
        "Robot_1_Storage_to_Observation" : 5.0,
    },

    "positions" : [
        # Robot_0's positions
        [
            [0.2028789492464582, 0.1698822601182645, 0.13624773367036977, 0.03513008510678816, 1.4919714064477891, -0.011501923089995283],    # PlaceConveyor
            [-0.06454724202416068, -0.25841679055839906, 0.07169377341193187, -0.5973192719933085, 1.4841716980991895, 2.5321741288187907],   # PlaceStorage
            [0.2762497323855258, -0.0758582449018067, 0.25877878754192806, -0.3753629275811858, 1.502019805732354, -0.41621094014756393],     # ObservationPositionConveyor
            [-0.017141437047257388, -0.15184164134413197, 0.2520253091903325, 1.019215447569528, 1.492753388202436, -1.5155268392165837]      # ObservationPositionStorage
        ],

        # Robot_1's positions
        [
            [0.2157542042065579, 0.20369258396709879, 0.1216293024165304, 2.158089291524011, 1.524596107925453, 2.2656189128608912],          # PlaceConveyor
            [-0.0017103279953780066, 0.2629236782267381, 0.07237643418601386, 0.6202080849782293, 1.5623164426206442, 2.26682857594807],      # PlaceStorage
            [0.28652238179577555, -0.07686871280532333, 0.2595967526715013, 0.13276838676211575, 1.4864162505849923, -0.08733655724281332],   # ObservationPositionConveyor
            [-0.026088641405653677, 0.15325372778995128, 0.25126163523766603, -0.47224216359848475, 1.4558342950810048, 1.3648727293344325]   # ObservationPositionStorage
        ],
    ],

    "Conveyor_workspace_0" : [
        [0.3596323585708997, -0.16060691865501742, 0.0879232666464339, -0.8482838203073986, 1.44674475818115, -0.9809107414147195], # Point 1
        [0.2656025776458838, -0.16234248511672184, 0.08981466599025785, -0.8731423886864879, 1.4465201025855459, -1.078532742812569], # Point 2
        [0.27112336642013984, -0.010876047358195792, 0.09140285537738904, -1.5987659897877258, 1.5161858673958504, -1.361278887462221], # Point 3
        [0.364236422169028, -0.009897310810647254, 0.0884393513217729, -0.6430197892426884, 1.4759699457773374, -0.9708794180681989]  # Point 4
    ],

    "Storage_workspace_0" : [
        [-0.103466899000137, -0.27441250785887167, 0.012661660402346273, -2.1019524133819165, 1.5367089101869544, 2.2474003130181446], # Point 1
        [-0.09793913504600177, -0.14392681286308606, 0.014045824358010453, 2.604594284904857, 1.4735611249636733, 0.6654300676483247], # Point 2
        [0.03906344207187781, -0.15329425030583527, 0.013850361180210213, 2.7166682708520398, 1.4161641902461588, 1.6199317959309736], # Point 3
        [0.028690639018074147, -0.28272223655170275, 0.014632981789200611, -0.8600034749690095, 1.4763606489287286, -2.086309577151151]  # Point 4
    ],

    "Conveyor_workspace_1" : [
        [0.37784148028150427, -0.1632706789540773, 0.08700622695598986, 2.2734846628180896, 1.4771489277284042, 1.8402898331902433], # Point 1
        [0.28989430371926733, -0.16083077822463437, 0.08871539459890214, 2.1831199414568925, 1.5080756213076822, 1.3256450779952678], # Point 2
        [0.2997813986743835, -0.005108734210596587, 0.08884020167662281, 2.1121545258888146, 1.5263725476759695, 1.7289873279623242], # Point 3
        [0.3859028493355478, -0.007854632943308088, 0.08612839514673899, 2.011836913228264, 1.5625274409612158, 1.7205162711741384]  # Point 4
    ],

    "Storage_workspace_1" : [
        [-0.10058191168910334, 0.16154865763136206, 0.01650482978725408, -1.2499469220381743, 1.56305438336274, -0.7345335041950191], # Point 3
        [0.03718107352065722, 0.1618793032131123, 0.016595891541987484, -1.4153997530676132, 1.5149205178866925, -1.6819281888854605],# Point 2
        [0.03211398591120434, 0.2932694995681843, 0.01787590517238276, -2.905693483888061, 1.5211577023579788, -2.843704932913777], # Point 1
        [-0.10069238343294361, 0.29318653824249463, 0.016166038666620655, -2.357499867376823, 1.5162626417225171, -2.070090724804433],# Point 4]
     ]
}
