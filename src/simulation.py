import pybullet as p
import pybullet_data
import time
import math

def start(controller_queue, simulation_queue):
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
    while True:
        if simulation_queue.empty():
            continue  # Skip if no message is received
        message = simulation_queue.get()
        print(f"Simulation: Received Queue Message: {message}")
        if message["type"] == "stop":
            print("Simulation: Stopping simulation.")
            break
        elif message["type"] == "set_axis_angle":
            axis_angle_degrees = message["axis_angle_degrees"]
            axis_name = message["axis_name"]

            axis_id = None
            match axis_name:
                case "R":
                    axis_id = R_AXIS_JOINT_IDX
                case "A":
                    axis_id = A_AXIS_JOINT_IDX
                case "B":
                    axis_id = B_AXIS_JOINT_IDX
                case "C":
                    axis_id = C_AXIS_JOINT_IDX
                case _:
                    print(f"Invalid axis letter: {message['axis_letter']}")
                    continue

            axis_angle_rad = math.radians(axis_angle_degrees)
            p.setJointMotorControl2(arm_id, axis_id, p.POSITION_CONTROL, targetPosition=axis_angle_rad)

            desired_angle_reached = False
            start_time = time.time()
            timeout_duration_seconds = 5
            while time.time() - start_time < timeout_duration_seconds:
                p.stepSimulation()
                time.sleep(time_step)

                # Check if the joint angle is within a small epsilon of the target angle
                joint_state = p.getJointState(arm_id, axis_id)
                current_angle_rad = joint_state[0]
                current_angle_deg = math.degrees(current_angle_rad)

                if abs(current_angle_deg - axis_angle_degrees) < 0.15:
                    desired_angle_reached = True
                    break
            
            
            message = {
                    "axis_angle_degrees": axis_angle_degrees,
                    "axis_name": axis_name,
                    "status": "success" if desired_angle_reached else "failure"
            }
            
            if desired_angle_reached:
                print(f"Simulation: Successfully reached target angle: {axis_angle_degrees} degrees for axis {axis_name}")
            else:
                print(f"Simulation: Failed to reach target angle: {axis_angle_degrees} degrees for axis {axis_name}")

            # Send the message back to the controller
            controller_queue.put(message)

            p.stepSimulation()
            time.sleep(time_step)