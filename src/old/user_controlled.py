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

# Add user sliders
r_axis_slider = p.addUserDebugParameter(" R-Axis Angle (deg)", 0, 360, 0)
a_axis_slider = p.addUserDebugParameter(" A-Axis Angle (deg)", -90, 90, 0)
b_axis_slider = p.addUserDebugParameter(" B-Axis Angle (deg)", 0, 160, 0)
#c_axis_slider = p.addUserDebugParameter(" C-Axis Angle (deg)", 0, 160, 0)

# Joint indices
# Fixed joint index (not used in control)
FIXED_JOINT_IDX = 1

R_AXIS_JOINT_IDX = 0
A_AXIS_JOINT_IDX = 2
B_AXIS_JOINT_IDX = 3
C_AXIS_JOINT_IDX = 4

# Infinite simulation loop
while True:
    r_axis_deg = p.readUserDebugParameter(r_axis_slider)
    a_axis_deg = p.readUserDebugParameter(a_axis_slider)
    b_axis_deg = p.readUserDebugParameter(b_axis_slider)
  #  c_axis_deg = p.readUserDebugParameter(c_axis_slider)

    r_axis_rad = math.radians(r_axis_deg)
    a_axis_rad = math.radians(a_axis_deg)
    b_axis_rad = math.radians(b_axis_deg)
  #  c_axis_rad = math.radians(c_axis_deg)

    p.setJointMotorControl2(arm_id, R_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=r_axis_rad)
    p.setJointMotorControl2(arm_id, A_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=a_axis_rad)
    p.setJointMotorControl2(arm_id, B_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=b_axis_rad)
  #  p.setJointMotorControl2(arm_id, C_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=c_axis_rad)

    p.stepSimulation()
    time.sleep(time_step)
