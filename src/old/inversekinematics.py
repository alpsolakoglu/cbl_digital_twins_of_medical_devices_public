import pybullet as p
import pybullet_data
import math
import time

# Start PyBullet with GUI
physics_client = p.connect(p.SHARED_MEMORY)
if physics_client < 0:
    physics_client = p.connect(p.GUI)

# Setup simulation environment
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# Load the floor and the robot arm URDF
plane_id = p.loadURDF("plane.urdf")
p.changeVisualShape(plane_id, -1, rgbaColor=[0.8, 0.8, 0.8, 1])

arm_id = p.loadURDF("/src/urdf/4dof.urdf")
p.resetBasePositionAndOrientation(arm_id, [0, 0, 0], [0, 0, 0, 1])
arm_end_effector_index = 4

# Set a distinct base color for the robot arm
p.changeVisualShape(arm_id, -1, rgbaColor=[0.1, 0.1, 0.8, 1])

num_joints = p.getNumJoints(arm_id)
print(f"Number of joints in URDF: {num_joints}")

# Store information about all movable joints that PyBullet's IK solver will control
movable_joint_indices = []
lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []

# Default rest pose values for your 4 DoF arm.
default_rest_pose_values = [math.pi / 2, 0.0, 0.1, 0.1] 

current_rest_pose_idx = 0
for i in range(num_joints):
    joint_info = p.getJointInfo(arm_id, i)
    joint_index = joint_info[0]
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]
    
    # Filter for revolute or prismatic joints.
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        movable_joint_indices.append(joint_index)
        ll = joint_info[8]
        ul = joint_info[9]
        
        lower_limits.append(ll)
        upper_limits.append(ul)
        joint_ranges.append(ul - ll)
        
        if current_rest_pose_idx < len(default_rest_pose_values):
            rest_poses.append(default_rest_pose_values[current_rest_pose_idx])
        else:
            rest_poses.append((ll + ul) / 2.0)
        current_rest_pose_idx += 1
        
        print(f"Movable Joint: {joint_name} (Index: {joint_index})")
        print(f"  Lower Limit: {ll:.2f}, Upper Limit: {ul:.2f}, Range: {joint_ranges[-1]:.2f}, Rest Pose: {rest_poses[-1]:.2f}")

nDOf = len(movable_joint_indices)
print(f"\nParameters for calculateInverseKinematics ({nDOf} DoF):")
print(f"lowerLimits (ll): {lower_limits}")
print(f"upperLimits (ul): {upper_limits}")
print(f"jointRanges (jr): {joint_ranges}")
print(f"restPoses (rp): {rest_poses}")

# --- Target Position for End-Effector (initially fixed, now dynamic) ---
# This is the 3D Cartesian position [x, y, z] that the end-effector will try to reach.
link_state = p.getLinkState(arm_id, arm_end_effector_index)
target_position = list(link_state[4])  # linkWorldPosition

# Define how much the target position changes with each simulation step while key is pressed
target_step_size = 0.01

print("\n--- Keyboard Controls ---")
print("Move the End-Effector Target Position:")
print("  W/S: +X/-X (forward/backward) - Hold to move continuously")
print("  A/D: +Y/-Y (left/right) - Hold to move continuously")
print("  Q/E: +Z/-Z (up/down) - Hold to move continuously")
print("--------------------------\n")


# Main simulation loop
try:
    print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")

    while p.isConnected():
        # --- Handle Keyboard Input for Target Position ---
        keys = p.getKeyboardEvents() 

        if ord('w') in keys and (keys[ord('w')] & p.KEY_IS_DOWN):
            target_position[0] += target_step_size
            print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        if ord('s') in keys and (keys[ord('s')] & p.KEY_IS_DOWN):
            target_position[0] -= target_step_size
            print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:0.3f}, {target_position[2]:.3f}]")
        
        if ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN):
            target_position[1] += target_step_size
            print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        if ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN):
            target_position[1] -= target_step_size
            print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        
        if ord('e') in keys and (keys[ord('e')] & p.KEY_IS_DOWN):
            target_position[2] += target_step_size
            print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        if ord('q') in keys and (keys[ord('q')] & p.KEY_IS_DOWN):
            target_position[2] -= target_step_size
            print(f"Current Target Position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")

        # --- Calculate Inverse Kinematics ---
        joint_poses = p.calculateInverseKinematics(
            arm_id,
            arm_end_effector_index,
            target_position,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
            solver=p.IK_DLS
        )

        # --- Apply Joint Control if IK Solution Found ---
        if joint_poses:
            for i in range(nDOf):
                p.setJointMotorControl2(
                    bodyIndex=arm_id,
                    jointIndex=movable_joint_indices[i],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_poses[i],
                    maxVelocity=1.0
                )
        else:
            pass # Suppress repeated messages if the target is continuously unreachable

        p.stepSimulation()
        time.sleep(time_step)

except KeyboardInterrupt:
    print("Simulation interrupted by user (KeyboardInterrupt).")
except Exception as e:
    print(f"An error occurred during simulation: {e}")
finally:
    p.disconnect()
    print("PyBullet simulation ended.")
