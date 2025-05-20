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
arm_start_pos = [0, 0, 0.1]
arm_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
arm_id = p.loadURDF("urdf/simple2d_arm.urdf", arm_start_pos, arm_start_orientation, useFixedBase=True)

# Add user debug sliders
pitch_slider = p.addUserDebugParameter("Camera Pitch", -90, 90, -10)
joint1_slider = p.addUserDebugParameter("Joint 1 Angle (deg)", -180, 180, 90)
joint2_slider = p.addUserDebugParameter("Joint 2 Angle (deg)", -180, 180, 90)

# Joint indices
LINK1_JOINT_IDX = 0
LINK2_JOINT_IDX = 1

# Infinite simulation loop
while True:
    # Read slider values
    pitch = p.readUserDebugParameter(pitch_slider)
    joint1_deg = p.readUserDebugParameter(joint1_slider)
    joint2_deg = p.readUserDebugParameter(joint2_slider)

    # Update camera pitch
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=180,
        cameraPitch=pitch,
        cameraTargetPosition=[0, 0, 1.0]
    )

    # Convert to radians and apply position control
    joint1_rad = math.radians(joint1_deg)
    joint2_rad = math.radians(joint2_deg)

    p.setJointMotorControl2(arm_id, LINK1_JOINT_IDX, p.POSITION_CONTROL, targetPosition=joint1_rad)
    p.setJointMotorControl2(arm_id, LINK2_JOINT_IDX, p.POSITION_CONTROL, targetPosition=joint2_rad)

    p.stepSimulation()
    time.sleep(time_step)
