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
arm_id = p.loadURDF("/src/urdf/4dof.urdf", arm_start_pos, arm_start_orientation, useFixedBase=True)

# Add user sliders
r_axis_slider = p.addUserDebugParameter("R-Axis Angle (deg)", 0, 180, 0)

# Joint indices
R_AXIS_JOINT_IDX = 0
A_AXIS_JOINT_IDX = 1
B_AXIS_JOINT_IDX = 2
C_AXIS_JOINT_IDX = 3

# Infinite simulation loop
while True:
    r_axis_deg = p.readUserDebugParameter(r_axis_slider)

    r_axis_rad = math.radians(r_axis_deg)

    p.setJointMotorControl2(arm_id, R_AXIS_JOINT_IDX, p.POSITION_CONTROL, targetPosition=r_axis_rad)

    p.stepSimulation()
    time.sleep(time_step)
