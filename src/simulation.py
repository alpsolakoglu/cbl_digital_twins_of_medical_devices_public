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

    gallbladder_position = [-1.1, -0.2, 0] # Position it where you want
    gallbladder_orientation = p.getQuaternionFromEuler([0, 0, 1.5])

    gallbladder_id = p.loadURDF(
        "src/urdf/gallbladder.urdf",
        basePosition=gallbladder_position,
        baseOrientation=gallbladder_orientation,
        useFixedBase=True # Makes the object static
    )


    # Load the robot arm
    arm_start_pos = [0, 0, 0]
    arm_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    arm_id = p.loadURDF("/src/urdf/onshape.urdf", arm_start_pos, arm_start_orientation, useFixedBase=True)

    # Joint indices
    # Fixed joint index (not used in control)
    FIXED_JOINT_IDX = 0

    R_AXIS_JOINT_IDX = 1
    C_AXIS_JOINT_IDX = 2
    B_AXIS_JOINT_IDX = 3
    A_AXIS_JOINT_IDX = 4

    # Infinite simulation loop
    while True:
        while not simulation_queue.empty():
            message = simulation_queue.get()
            print(f"Simulation: Received Queue Message: {message}")
            if message["type"] == "stop":
                controller_queue.put({"type": "stop"})
                print("Simulation: Stopping simulation.")
                break
            elif message["type"] == "twin_axis_angle":
                axis_name = message["axis_name"]
                angle_degrees = message["axis_angle_degrees"]

                if angle_degrees > 180:
                    angle_degrees = angle_degrees - 360  # Normalize to [-180, 180]

                joint_index = None
                if axis_name == "R":
                    joint_index = R_AXIS_JOINT_IDX
                elif axis_name == "A":
                    joint_index = A_AXIS_JOINT_IDX
                elif axis_name == "B":
                    joint_index = B_AXIS_JOINT_IDX
                elif axis_name == "C":
                    joint_index = C_AXIS_JOINT_IDX

                p.setJointMotorControl2(
                    arm_id, 
                    joint_index, 
                    p.POSITION_CONTROL, 
                    targetPosition=angle_degrees * math.pi / 180.0  # Convert degrees to radians
                )

        # Check if height of last effector index is below a threshold
        # linkState = p.getLinkState(arm_id, A_AXIS_JOINT_IDX)
        # framePos = linkState[4]
        # frameOrientation = linkState[5]
                                     
        # roll, pitch, yaw = p.getEulerFromQuaternion(frameOrientation)



        # print(f"framePos:")
        # print(framePos)
        # print(f"frameOreinetation: ")
        # print(frameOrientation)

        p.stepSimulation()
        time.sleep(time_step)