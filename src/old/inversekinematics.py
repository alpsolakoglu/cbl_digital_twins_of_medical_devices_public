import pybullet as p
import pybullet_data
import math
import time

# Start PyBullet with GUI
physics_client = p.connect(p.SHARED_MEMORY)
if physics_client < 0:
    p.connect(p.GUI)

# Setup simulation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# Load the floor
p.loadURDF("plane.urdf")
arm_id = p.loadURDF("src/urdf/onshape.urdf", basePosition=[0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(arm_id, [0, 0, 0], [0, 0, 0, 1])
arm_end_effector_index = 4
TARGET_POSITION = [-1.6, 0, 0.2]


gallbladder_position = [-1.5, 0, 0.2] 
gallbladder_orientation = p.getQuaternionFromEuler([0, 0, 1.5])

gallbladder_id = p.loadURDF(
    "src/urdf/gallbladder.urdf",
    basePosition=gallbladder_position,
    baseOrientation=gallbladder_orientation,
    useFixedBase=True # Makes the object static
)
# # Helper to draw the local axes of the end-effector link
# p.addUserDebugLine([0,0,0], [0.1,0,0], [1,0,0], parentObjectUniqueId=arm_id, parentLinkIndex=arm_end_effector_index) # X-axis (Red)
# p.addUserDebugLine([0,0,0], [0,0.1,0], [0,1,0], parentObjectUniqueId=arm_id, parentLinkIndex=arm_end_effector_index) # Y-axis (Green)
# p.addUserDebugLine([0,0,0], [0,0,0.1], [0,0,1], parentObjectUniqueId=arm_id, parentLinkIndex=arm_end_effector_index) # Z-axis (Blue)

target_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1])
p.createMultiBody(baseVisualShapeIndex=target_visual_shape, basePosition=TARGET_POSITION)

num_joints = p.getNumJoints(arm_id)
print(f"Number of joints in URDF: {num_joints}")

# Store information about all movable joints
movable_joint_indices = []
lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []

for i in range(num_joints):
    joint_info = p.getJointInfo(arm_id, i)
    if joint_info[2] == p.JOINT_REVOLUTE:
        movable_joint_indices.append(i)
        lower_limits.append(joint_info[8])
        upper_limits.append(joint_info[9])
        joint_ranges.append(joint_info[9] - joint_info[8])


while p.isConnected():

    current_joint_states = p.getJointStates(arm_id, movable_joint_indices)
    current_joint_positions = [state[0] for state in current_joint_states]

    joint_poses = p.calculateInverseKinematics(
        arm_id,
        arm_end_effector_index,
        TARGET_POSITION,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=current_joint_positions
    )

    print(f"Calculated joint poses: {joint_poses}")

    for i in range(len(movable_joint_indices)):
        p.setJointMotorControl2(
            bodyIndex=arm_id,
            jointIndex=movable_joint_indices[i],
            controlMode=p.POSITION_CONTROL, 
            targetPosition=joint_poses[i],
            positionGain=0.05, 
            velocityGain=1.0, 
            force=5000
        )

    p.stepSimulation()
    time.sleep(time_step )

p.disconnect()
print("PyBullet simulation ended.")
