from dataclasses import dataclass
from typing import Literal

from pyniryo import ObjectColor, ObjectShape


@dataclass
class StorageObject:
    name: str
    shape: ObjectShape
    color: ObjectColor
    position: Literal["Storage_1", "In_Transit", "Storage_0"]


configuration = {
    # IP's of the niryo ned2 robots
    "ips": ["169.254.200.200", "169.254.200.201"],
    # Order of execution for the robot arms
    "NumberOfPriorities": 2,
    "PickFromStoragePriority": 2,
    "PickFromIRSensorPriority": 1,
    # Overview of equipment in the experimental setup
    "NumberOfRobotArms": 2,
    # Dictionary over anomalies
    "Anomalies": {
        1: "Anomaly 1: Object pushed backwards on conveyor belt",
        2: "Anomaly 2: Object pushed forwards on conveyor belt",
        3: "Anomaly 3: Robot arm fails to pickup object from storage",
        4: "Anomaly 4: Robot arm fails to pickup object from conveyor",
        5: "Anomaly 5: Wrong object detected on conveyor belt",
        6: "Anomaly 6: Unidentified object on conveyor belt",
        7: "Anomaly 7: Conveyor belt does not move in the right direction with theright speed",
        8: "Anomaly 8: Object falls off conveyor belt",
        9: "Anomaly 9: Object falls off robot arm",
        10: "Anomaly 10: Object isn't released from vacuum",
        11: "Anomaly 11: Anomaly 11: Storage is full",
        12: "Anomaly 12: Unidentified object in storage",
        13: "Anomaly 13: Wrong object in storage",
        14: "Anomaly 14: Missing object in storage",
    },
    # Starting setup of the system
    "StorageObjects": [
        StorageObject("Red Square", ObjectShape.SQUARE, ObjectColor.RED, "Storage_1"),
        StorageObject("Blue Square", ObjectShape.SQUARE, ObjectColor.BLUE, "Storage_0"),
        StorageObject("Green Square", ObjectShape.SQUARE, ObjectColor.GREEN, "Storage_1"),
        StorageObject("Red Circle", ObjectShape.CIRCLE, ObjectColor.RED, "Storage_0"),
        StorageObject("Blue Circle", ObjectShape.CIRCLE, ObjectColor.BLUE, "Storage_1"),
        StorageObject("Green Circle", ObjectShape.CIRCLE, ObjectColor.GREEN, "Storage_0"),
    ],
    "StorageOccupancy": [
        [(ObjectShape.SQUARE, ObjectColor.BLUE), (ObjectShape.CIRCLE, ObjectColor.RED), (ObjectShape.CIRCLE, ObjectColor.GREEN), None],
        [(ObjectShape.SQUARE, ObjectColor.RED), (ObjectShape.SQUARE, ObjectColor.GREEN), (ObjectShape.CIRCLE, ObjectColor.BLUE), None],
    ],
    # Camera configurations found using experiments
    "brightness": [[1.24816061, 1.32431245], [1.28829838, 1.42241655]],
    "contrast": [[1.01757065, 1.02363494], [1.02113107, 1.01575003]],
    "saturation": [[1.23197705, 1.35534524], [1.27477756, 1.22645163]],
    # Times for the DT
    "Observed_times": {
        "Conveyor_0": 8.26,
        "Conveyor_1": 8.26,
        "Robot_0_Observation_to_Pickup_Storage": 8.86,
        "Robot_1_Observation_to_Pickup_Storage": 9.74,
        "Robot_0_Workspace_Observation_to_Pickup_Conveyor": 4.66,
        "Robot_1_Workspace_Observation_to_Pickup_Conveyor": 4.51,
        "Robot_0_Pickup_Conveyor_to_Observation": 4.16,
        "Robot_1_Pickup_Conveyor_to_Observation": 4.14,
        "Robot_0_Observation_to_Standby": 2.78,
        "Robot_1_Observation_to_Standby": 2.88,
        "Robot_0_Observation_to_Place_Storage": 3.35,
        "Robot_1_Observation_to_Place_Storage": 3.86,
        "Robot_0_Standby_to_Place_Conveyor": 1.87,
        "Robot_1_Standby_to_Place_Conveyor": 1.69,
        "Robot_0_Storage_to_Standby": 6.6,
        "Robot_1_Storage_to_Standby": 6.8,
        "Robot_0_Place_Storage_to_Observation": 4.18,
        "Robot_1_Place_Storage_to_Observation": 4.43,
        "Robot_0_Place_Conveyor_to_Observation": 3.43,
        "Robot_1_Place_Conveyor_to_Observation": 3.92,
        "Robot_0_Observation_to_Workspace_Observation": 100,
        "Robot_1_Observation_to_Workspace_Observation": 100,
    },
    "storagePositions": [
        [
            [-0.06205056905277242, -0.19256394859795326, 0.057984999202619444, 1.770515246141111, 1.548892970661883, -1.4110397860855048],
            [0.005508972910148029, -0.19919787963968721, 0.05732826415965928, 2.6028315915415394, 1.558259000508785, -0.6029308938909559],
            [-0.0673181900989941, -0.25935795259543687, 0.056629867433290425, 2.716193629686612, 1.5455437127533735, -0.7021146089214897],
            [0.0007075202392178714, -0.2599278711928543, 0.056952659324702594, 2.2527964034233254, 1.5296351721639505, -0.9329650414391197],
        ],
        [
            [-0.00282078630144435, 0.19717361034206676, 0.06299425747143081, -2.539337643624458, 1.5674382910114895, 3.0487939997302145],
            [-0.07824856778361586, 0.19160351414658516, 0.05385308002668253, -2.3876118714603307, 1.553386806421733, -2.8703027663285465],
            [-0.007873883105455861, 0.2633992356019009, 0.05557682514481014, 3.100216674723426, 1.4980368823362151, 2.2528459022634792],
            [-0.0792910869080455, 0.2581088096320052, 0.051502537161709855, -2.2332863270209296, 1.5054596853471967, -2.829187564225824],
        ],
    ],
    "positions": [
        # Robot_0's positions
        [
            [
                0.2028789492464582,
                0.1698822601182645,
                0.13624773367036977,
                0.03513008510678816,
                1.4919714064477891,
                -0.011501923089995283,
            ],  # PlaceConveyor

            [
                0.22588446311184643,
                -0.0008558529969849022,
                0.5181733234906416,
                0.036411234992297405,
                1.1864229848405983,
                0.03565316230539174
            ],  # Observation

            [
                -0.017141437047257388,
                -0.15184164134413197,
                0.2520253091903325,
                1.019215447569528,
                1.492753388202436,
                -1.5155268392165837,
            ],  # ObservationPositionStorage
            [
                0.2028789492464582,
                0.1698822601182645,
                0.23624773367036977,
                0.03513008510678816,
                1.4919714064477891,
                -0.011501923089995283,
            ],  # Standby Position

             [
                0.2762497323855258,
                -0.0758582449018067,
                0.25877878754192806,
                -0.3753629275811858,
                1.502019805732354,
                -0.41621094014756393,
            ] # Conveyor Workspace
        ],
        # Robot_1's positions
        [
            [
                0.2157542042065579,
                0.20369258396709879,
                0.1216293024165304,
                2.158089291524011,
                1.524596107925453,
                2.2656189128608912,
            ],  # PlaceConveyor

            [
                0.2058824271247155,
                0.0007054255360600889,
                0.5307552368864079,
                -0.028579226480643805,
                1.11042505722123,
                -0.03175794465104596
            ], # Observation

            [
                -0.026088641405653677,
                0.15325372778995128,
                0.25126163523766603,
                -0.47224216359848475,
                1.4558342950810048,
                1.3648727293344325,
            ],  # ObservationPositionStorage
            [
                0.2157542042065579,
                0.20369258396709879,
                0.2216293024165304,
                2.158089291524011,
                1.524596107925453,
                2.2656189128608912,
            ],  # Standby Position

            [
                0.28652238179577555,
                -0.07686871280532333,
                0.2595967526715013,
                0.13276838676211575,
                1.4864162505849923,
                -0.08733655724281332,
            ], # Conveyor Workspace

        ],
    ],
    "Conveyor_workspace_0": [
        [
            0.3596323585708997,
            -0.16060691865501742,
            0.0879232666464339,
            -0.8482838203073986,
            1.44674475818115,
            -0.9809107414147195,
        ],  # Point 1
        [
            0.2656025776458838,
            -0.16234248511672184,
            0.08981466599025785,
            -0.8731423886864879,
            1.4465201025855459,
            -1.078532742812569,
        ],  # Point 2
        [
            0.27112336642013984,
            -0.010876047358195792,
            0.09140285537738904,
            -1.5987659897877258,
            1.5161858673958504,
            -1.361278887462221,
        ],  # Point 3
        [
            0.364236422169028,
            -0.009897310810647254,
            0.0884393513217729,
            -0.6430197892426884,
            1.4759699457773374,
            -0.9708794180681989,
        ],  # Point 4
    ],
    "Storage_workspace_0": [
        [
            -0.103466899000137,
            -0.27441250785887167,
            0.012661660402346273,
            -2.1019524133819165,
            1.5367089101869544,
            2.2474003130181446,
        ],  # Point 1
        [
            -0.09793913504600177,
            -0.14392681286308606,
            0.014045824358010453,
            2.604594284904857,
            1.4735611249636733,
            0.6654300676483247,
        ],  # Point 2
        [
            0.03906344207187781,
            -0.15329425030583527,
            0.013850361180210213,
            2.7166682708520398,
            1.4161641902461588,
            1.6199317959309736,
        ],  # Point 3
        [
            0.028690639018074147,
            -0.28272223655170275,
            0.014632981789200611,
            -0.8600034749690095,
            1.4763606489287286,
            -2.086309577151151,
        ],  # Point 4
    ],
    "Conveyor_workspace_1": [
        [
            0.37784148028150427,
            -0.1632706789540773,
            0.08700622695598986,
            2.2734846628180896,
            1.4771489277284042,
            1.8402898331902433,
        ],  # Point 1
        [
            0.28989430371926733,
            -0.16083077822463437,
            0.08871539459890214,
            2.1831199414568925,
            1.5080756213076822,
            1.3256450779952678,
        ],  # Point 2
        [
            0.2997813986743835,
            -0.005108734210596587,
            0.08884020167662281,
            2.1121545258888146,
            1.5263725476759695,
            1.7289873279623242,
        ],  # Point 3
        [
            0.3859028493355478,
            -0.007854632943308088,
            0.08612839514673899,
            2.011836913228264,
            1.5625274409612158,
            1.7205162711741384,
        ],  # Point 4
    ],
    "Storage_workspace_1": [
        [0.017317300935056065, 0.3002124567739622, 0.03511948402282107, 2.6378339691142987, 1.4997496448438217, -2.158278516800187],  # Point 3
        [0.025728889297184226, 0.1637783519383701, 0.03653153739400475, -2.702986682551672, 1.5025317697712444, -1.4800880927813516],  # Point 2
        [-0.11050817985167179, 0.156512995812484, 0.03286177904171689, -2.979056901500458, 1.488942137437697, -1.1083414664368054],  # Point 1
        [-0.1209067926198649, 0.28474806063597224, 0.03176011025672984, 2.5892731970701384, 1.4986216949749003, -1.887249324684688],  # Point 4]
    ],
}
