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

# Joint indices
# Fixed joint index (not used in control)
FIXED_JOINT_IDX = 1

R_AXIS_JOINT_IDX = 0
A_AXIS_JOINT_IDX = 2
B_AXIS_JOINT_IDX = 3
C_AXIS_JOINT_IDX = 4

# Infinite simulation loop

# Placeholder for message received check
message_received = True  
while True:
    # Check is message received
    # If message is received, check format of the message and return error if invalid, otherewise update the joint angle
    # After updating the joint angle, step the simulation until the desired angle is reached (with some epsilon)
    # If the angle is reached, and no constraints are violate during any step, send message back to the controller with success, and failure otherwise
    if message_received:
        angle_deg= 180
        joint = R_AXIS_JOINT_IDX

        angle_rad = math.radians(angle_deg)
        p.setJointMotorControl2(arm_id, joint, p.POSITION_CONTROL, targetPosition=angle_rad)


        desired_angle_reached = False
        start_time = time.time()
        timeout_duration_seconds = 5
        while time.time() - start_time < timeout_duration_seconds:
            p.stepSimulation()
            time.sleep(time_step)

            # Check if the joint angle is within a small epsilon of the target angle
            joint_state = p.getJointState(arm_id, joint)
            current_angle_rad = joint_state[0]
            current_angle_deg = math.degrees(current_angle_rad)

            if abs(current_angle_deg - angle_deg) < 0.1:
                desired_angle_reached = True
                break

        if desired_angle_reached:
            print(f"Successfully reached target angle: {angle_deg} degrees for joint {joint}")
            # Send success message back to controller (placeholder)
            # send_message_to_controller("success")
        else:
            print(f"Failed to reach target angle: {angle_deg} degrees for joint {joint}")
            # Send failure message back to controller (placeholder)
            # send_message_to_controller("failure")

        message_received = False  # Reset after processing

    p.stepSimulation()
    time.sleep(time_step)


