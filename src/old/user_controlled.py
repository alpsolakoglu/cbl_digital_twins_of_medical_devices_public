import pybullet as p
import pybullet_data
import time
import math

# Start PyBullet with GUI
physics_client = p.connect(p.GUI)

# Setup simulation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# Load the floor
p.loadURDF("plane.urdf")

# Load the robot arm
arm_start_pos = [0, 0, 0]
arm_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
arm_id = p.loadURDF("/src/urdf/onshape.urdf", arm_start_pos, arm_start_orientation, useFixedBase=True)

gallbladder_position = [-1.5, 0.3, 0.2] # Position it where you want
gallbladder_orientation = p.getQuaternionFromEuler([0, 0, 1.5])

gallbladder_id = p.loadURDF(
    "src/urdf/gallbladder.urdf",
    basePosition=gallbladder_position,
    baseOrientation=gallbladder_orientation,
    useFixedBase=True # Makes the object static
)

# Add user sliders
r_axis_slider = p.addUserDebugParameter(" R-Axis Angle (deg)", -60, 60, 0)
c_axis_slider = p.addUserDebugParameter(" C-Axis Angle (deg)", -60, 80, 0)
b_axis_slider = p.addUserDebugParameter(" B-Axis Angle (deg)", 0, 160, 0)
a_axis_slider = p.addUserDebugParameter(" A-Axis Angle (deg)", -90, 90, 0)

# Joint indices
# Fixed joint index (not used in control)
FIXED_JOINT_IDX = 0

R_AXIS_JOINT_IDX = 1
C_AXIS_JOINT_IDX = 2
B_AXIS_JOINT_IDX = 3
A_AXIS_JOINT_IDX = 4

# JOINT_OFFSETS = {
#     R_AXIS_JOINT_IDX: 0.0,              
#     A_AXIS_JOINT_IDX: 0.55,     
#     B_AXIS_JOINT_IDX: 1.8,              
#     C_AXIS_JOINT_IDX: 0.8     
# }

# Infinite simulation loop
while True:
    r_axis_deg = p.readUserDebugParameter(r_axis_slider)
    c_axis_deg = p.readUserDebugParameter(c_axis_slider)
    b_axis_deg = p.readUserDebugParameter(b_axis_slider)
    a_axis_deg = p.readUserDebugParameter(a_axis_slider)

    r_axis_rad = math.radians(r_axis_deg)
    c_axis_rad = math.radians(c_axis_deg)
    b_axis_rad = math.radians(b_axis_deg)
    a_axis_rad = math.radians(a_axis_deg)

    # target_r_axis_rad = r_axis_rad + JOINT_OFFSETS[R_AXIS_JOINT_IDX]
    # target_a_axis_rad = a_axis_rad + JOINT_OFFSETS[A_AXIS_JOINT_IDX]
    # target_b_axis_rad = b_axis_rad + JOINT_OFFSETS[B_AXIS_JOINT_IDX]
    # target_c_axis_rad = c_axis_rad + JOINT_OFFSETS[C_AXIS_JOINT_IDX]

    target_r_axis_rad = r_axis_rad
    target_c_axis_rad = c_axis_rad      
    target_b_axis_rad = b_axis_rad
    target_a_axis_rad = a_axis_rad

    p.setJointMotorControl2(arm_id, R_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=target_r_axis_rad)
    p.setJointMotorControl2(arm_id, C_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=target_c_axis_rad)
    p.setJointMotorControl2(arm_id, B_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=target_b_axis_rad)
    p.setJointMotorControl2(arm_id, A_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=target_a_axis_rad)


    p.stepSimulation()
    time.sleep(time_step)
