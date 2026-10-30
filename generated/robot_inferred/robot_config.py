"""Generated EV3 control package (Stage 5.1).

Produced by ``python -m src.codegen.generate`` from the
inferred kinematic model. Abstract kinematics + code only:
no part IDs, poses or appearance data, so this package is
public. Bricks see ports only; joint names live on the PC
side.

DO NOT hand-edit: regenerate after any model or calibration
change and re-run the bring-up checks.

Robot-specific constants for robot_inferred: joints, coupling, actuators, guards, homing plan."""

ROBOT = 'robot_inferred'

FIRST_RUN_SPEED_DEG_S = 30.0  # ~30% of the 100 deg/s cap
SPEED_CAP_DEG_S = 100.0
SOFT_MARGIN_DEG = 5.0  # soft limits narrow the model limits

JOINT_IDS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
MOTOR_IDS = ['p000147', 'p000148', 'p000149', 'p000736', 'p000737', 'p000743']

JOINTS = [{'axis': [0.0, 0.0012, 1.0],
  'child': 'D1',
  'id': 'J1',
  'limit_source': {'hi': 'sensor', 'lo': 'collision'},
  'limits_deg': [-50.0, 39.0],
  'parent': 'D0',
  'point_mm': [21.76, 79.58, -139.78],
  'soft_limits_deg': [-45.0, 34.0]},
 {'axis': [0.707, 0.7072, -0.0001],
  'child': 'D2',
  'id': 'J2',
  'limit_source': {'hi': 'sensor', 'lo': 'collision'},
  'limits_deg': [-116.25, 40.0],
  'parent': 'D1',
  'point_mm': [21.77, 79.71, -53.02],
  'soft_limits_deg': [-111.25, 35.0]},
 {'axis': [0.707, 0.7072, 0.0],
  'child': 'D3',
  'id': 'J3',
  'limit_source': {'hi': 'sensor', 'lo': 'collision'},
  'limits_deg': [-74.38, 21.0],
  'parent': 'D2',
  'point_mm': [23.37, 78.06, 115.02],
  'soft_limits_deg': [-69.38, 16.0]},
 {'axis': [-0.7071, 0.7071, 0.0],
  'child': 'D4',
  'id': 'J4',
  'limit_source': {'hi': 'collision', 'lo': 'collision'},
  'limits_deg': [-115.0, 125.0],
  'parent': 'D3',
  'point_mm': [-37.39, 138.83, 162.38],
  'soft_limits_deg': [-110.0, 120.0]},
 {'axis': [0.0008, -0.0008, -1.0],
  'child': 'D5',
  'id': 'J5',
  'limit_source': {'hi': 'collision', 'lo': 'collision'},
  'limits_deg': [-90.0, 124.38],
  'parent': 'D4',
  'point_mm': [-82.56, 183.99, 162.22],
  'soft_limits_deg': [-85.0, 119.38]},
 {'axis': [-0.7068, 0.7074, 0.0005],
  'child': 'D6',
  'id': 'J6',
  'limit_source': {'hi': 'collision', 'lo': 'collision'},
  'limits_deg': [-60.0, 135.0],
  'parent': 'D5',
  'point_mm': [-115.29, 216.88, 162.33],
  'soft_limits_deg': [-55.0, 130.0]}]

C = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.714286, 0.0, 0.0]]  # q = C m, joint deg from motor deg

ACTUATORS = [{'brick': 'right', 'id': 'p000147', 'motor': 'medium', 'port': 'outB'},
 {'brick': 'left', 'id': 'p000148', 'motor': 'large', 'port': 'outB'},
 {'brick': 'left', 'id': 'p000149', 'motor': 'large', 'port': 'outA'},
 {'brick': 'left', 'id': 'p000736', 'motor': 'medium', 'port': 'outC'},
 {'brick': 'right', 'id': 'p000737', 'motor': 'medium', 'port': 'outA'},
 {'brick': 'left', 'id': 'p000743', 'motor': 'medium', 'port': 'outD'}]

ZERO_OFFSETS = {'p000147': 0.0,
 'p000148': 0.0,
 'p000149': 0.0,
 'p000736': 0.0,
 'p000737': 0.0,
 'p000743': 0.0}  # human calibration (homing.local.yaml)

GUARD_RULES = {'left': [{'invert': False, 'sensor': 'in1'},
          {'invert': False, 'sensor': 'in2'},
          {'invert': False, 'sensor': 'in3'}],
 'right': [{'invert': False, 'sensor': 'in1'},
           {'invert': False, 'sensor': 'in2'}]}

HOME_PLAN = [{'direction': 1,
  'direction_confirmed': False,
  'joint': 'J1',
  'method': 'local',
  'motor': 'p000147',
  'motor_brick': 'right',
  'motor_port': 'outB',
  'sensor': {'brick': 'right', 'port': 'in1', 'uid': 'p000375'}},
 {'direction': 1,
  'direction_confirmed': False,
  'joint': 'J2',
  'method': 'local',
  'motor': 'p000149',
  'motor_brick': 'left',
  'motor_port': 'outA',
  'sensor': {'brick': 'left', 'port': 'in1', 'uid': 'p000339'}},
 {'direction': 1,
  'direction_confirmed': False,
  'joint': 'J3',
  'method': 'local',
  'motor': 'p000148',
  'motor_brick': 'left',
  'motor_port': 'outB',
  'sensor': {'brick': 'left', 'port': 'in2', 'uid': 'p000630'}},
 {'direction': -1,
  'direction_confirmed': False,
  'joint': 'J4',
  'method': 'remote',
  'motor': 'p000737',
  'motor_brick': 'right',
  'motor_port': 'outA',
  'sensor': {'brick': 'left', 'port': 'in3', 'uid': 'p000796'}},
 {'direction': -1,
  'direction_confirmed': False,
  'joint': 'J5',
  'method': 'stall',
  'motor': 'p000736',
  'motor_brick': 'left',
  'motor_port': 'outC',
  'sensor': None},
 {'direction': -1,
  'direction_confirmed': False,
  'joint': 'J6',
  'method': 'stall',
  'motor': 'p000736',
  'motor_brick': 'left',
  'motor_port': 'outC',
  'sensor': None}]

EE_LINK = 'D6'

LIMIT_SOURCE = {'J1': {'hi': 'sensor', 'lo': 'collision'},
 'J2': {'hi': 'sensor', 'lo': 'collision'},
 'J3': {'hi': 'sensor', 'lo': 'collision'},
 'J4': {'hi': 'collision', 'lo': 'collision'},
 'J5': {'hi': 'collision', 'lo': 'collision'},
 'J6': {'hi': 'collision', 'lo': 'collision'}}  # per-side provenance: sensor/collision

COUPLED_LIMITS = [{'a': [1.0, 1.0], 'b': 27.2, 'joints': ['J2', 'J3'], 'op': '>='},
 {'a': [1.0, -1.0], 'b': 34.69, 'joints': ['J3', 'J5'], 'op': '>='},
 {'a': [1.0, 1.0], 'b': 206.74, 'joints': ['J2', 'J4'], 'op': '>='}]  # a1*qJa + a2*qJb + b >=/<= 0

COUPLED_LOOKUP = {}  # 'JA_JB' -> colliding grid cells
