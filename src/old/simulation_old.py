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

    JOINT_OFFSETS = {
        R_AXIS_JOINT_IDX: 0.0,
        A_AXIS_JOINT_IDX: 0.55,
        B_AXIS_JOINT_IDX: 1.8,
        C_AXIS_JOINT_IDX: 0.8
    }

    arm_end_effector_index = C_AXIS_JOINT_IDX
    num_joints = p.getNumJoints(arm_id)

    movable_joint_indices = []
    lower_limits = []
    upper_limits = []
    joint_ranges = []
    rest_poses = []

    for i in range(num_joints):
        joint_info = p.getJointInfo(arm_id, i)
        if joint_info[2] == p.JOINT_REVOLUTE:
            joint_index = joint_info[0]
            movable_joint_indices.append(joint_index)
            lower_limits.append(joint_info[8])
            upper_limits.append(joint_info[9])
            joint_ranges.append(joint_info[9] - joint_info[8])
            rest_poses.append(JOINT_OFFSETS.get(joint_index, 0))

    # Infinite simulation loop
    while True:
        if simulation_queue.empty():
            continue  # Skip if no message is received
        message = simulation_queue.get()
        print(f"Simulation: Received Queue Message: {message}")
        if message["type"] == "stop":
            controller_queue.put({"type": "stop"})
            print("Simulation: Stopping simulation.")
            break
        elif message["type"] == "set_all_angles":
                joint_angles_rad = message["angles_rad"]
                if len(joint_angles_rad) == len(movable_joint_indices):
                    for i, joint_index in enumerate(movable_joint_indices):
                        p.setJointMotorControl2(
                            arm_id, 
                            joint_index, 
                            p.POSITION_CONTROL, 
                            targetPosition=joint_angles_rad[i]
                        )

        joint_poses = p.calculateInverseKinematics(
                    arm_id,
                    arm_end_effector_index,
                    target_position,
                    lowerLimits=lower_limits,
                    upperLimits=upper_limits,
                    jointRanges=joint_ranges,
                    restPoses=rest_poses
                )
        if joint_poses:
                    ik_message = {
                        "type": "set_all_angles",
                        "angles_rad": joint_poses
                    }
                    simulation_queue.put(ik_message)
                    controller_queue.put(ik_message)
        else:
            print("Simulation: IK solution not found.")
            
        # Send the message back to the controller
        controller_queue.put(message)

        p.stepSimulation()
        time.sleep(time_step)