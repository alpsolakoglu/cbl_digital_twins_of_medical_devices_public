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
arm_id = p.loadURDF("/src/urdf/4dof.urdf")
p.resetBasePositionAndOrientation(arm_id, [0, 0, 0], [0, 0, 0, 1])
arm_end_effector_index = 4

num_joints = p.getNumJoints(arm_id)
print(f"Number of joints in URDF: {num_joints}")

# Store information about all movable joints
movable_joint_indices = []
lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []

default_rest_pose_values = [math.pi /2 , 0.0, 0.1, 0.1] 

current_rest_pose_idx = 0
for i in range(num_joints):
    joint_info = p.getJointInfo(arm_id, i)
    joint_index = joint_info[0]
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]
    q_index = joint_info[3] 
    
    # Filter for revolute joints, as these are controlled by IK
    if joint_type == p.JOINT_REVOLUTE:
        movable_joint_indices.append(joint_index)
        ll = joint_info[8]
        ul = joint_info[9]
        
        lower_limits.append(ll)
        upper_limits.append(ul)
        joint_ranges.append(ul - ll)
        
        if current_rest_pose_idx < len(default_rest_pose_values):
            rest_poses.append(default_rest_pose_values[current_rest_pose_idx])
        else:
            rest_poses.append((ll + ul) / 2.0)  # Fallback value
        current_rest_pose_idx += 1
        
        print(f"Movable Joint: {joint_name} (Index: {joint_index}, q_index: {q_index})")
        print(f"  Lower Limit: {ll}, Upper Limit: {ul}, Range: {joint_ranges[-1]}, Rest Pose: {rest_poses[-1]}")

# nDOf = len(movable_joint_indices)
# print(f"\nParameters for calculateInverseKinematics ({nDOf} DoF):")
# print(f"lowerLimits (ll): {lower_limits}")
# print(f"upperLimits (ul): {upper_limits}")
# print(f"jointRanges (jr): {joint_ranges}")
# print(f"restPoses (rp): {rest_poses}")
target_position = None

while p.isConnected():

    events = p.getMouseEvents()
    for event in events:
        #print(f"Event: {event}")
        if event[0] == 2:
            if (target_position is None):
                target_position = [event[1], event[2], 0]
                print(f"Target Position: {target_position}")

                joint_poses = p.calculateInverseKinematics(
                    arm_id,
                    arm_end_effector_index,
                    target_position,
                    lowerLimits=lower_limits,
                    upperLimits=upper_limits,
                    jointRanges=joint_ranges,
                    restPoses=rest_poses
                )
                print(f"Joint Poses: {joint_poses}")

                if joint_poses:   
                    for i in range(len(movable_joint_indices)):
                        p.setJointMotorControl2(
                            bodyIndex=arm_id,
                            jointIndex=movable_joint_indices[i],
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition=joint_poses[i],  
                            maxVelocity=1.0 
                        )
                else:
                    print("IK solution not found for the target position.")
                target_position = None

    p.stepSimulation()
    time.sleep(time_step)
