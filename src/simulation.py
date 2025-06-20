import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation

# def offset_position_with_quaternion(link_state, local_offset):
#     """
#     Offset a position using a quaternion rotation.
    
#     Args:
#         link_state: Your link state array (assuming position at [0:3] and quaternion at [5])
#         local_offset: 3D offset vector in local coordinates [x, y, z]
    
#     Returns:
#         new_position: The offset position in world coordinates
#     """
#     # Extract current position and quaternion from link_state
#     current_position = np.array(link_state[0:3])  # [x, y, z]
#     quaternion = np.array(link_state[5])  # Assuming this is [x, y, z, w] format
    
#     # Create rotation object from quaternion
#     # Note: scipy expects [x, y, z, w] format
#     rotation = R.from_quat(quaternion)
    
#     # Transform the local offset to world coordinates
#     world_offset = rotation.apply(local_offset)
    
#     # Add the transformed offset to current position
#     new_position = current_position + world_offset
    
#     return new_position

# # Alternative implementation if you prefer manual quaternion math
# def quaternion_rotate_vector(quat, vector):
#     """
#     Manually rotate a vector by a quaternion.
    
#     Args:
#         quat: quaternion [x, y, z, w]
#         vector: 3D vector [x, y, z]
    
#     Returns:
#         rotated_vector: The rotated 3D vector
#     """
#     # Extract quaternion components
#     qx, qy, qz, qw = quat
    
#     # Extract vector components
#     vx, vy, vz = vector
    
#     # Quaternion rotation formula
#     # v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
#     qxyz = np.array([qx, qy, qz])
#     v = np.array([vx, vy, vz])
    
#     cross1 = np.cross(qxyz, v)
#     cross2 = np.cross(qxyz, cross1 + qw * v)
    
#     rotated = v + 2 * cross2
#     return rotated

# def offset_position_manual(link_state, local_offset):
#     """
#     Manual implementation without scipy dependency.
#     """
#     current_position = np.array(link_state[4])
#     quaternion = np.array(link_state[5])  # [x, y, z, w]
    
#     # Rotate the offset vector
#     world_offset = quaternion_rotate_vector(quaternion, local_offset)
    
#     # Add to current position
#     new_position = current_position + world_offset
    
#     return new_position

def draw_circle(position, radius=0.1, color=[1, 0, 0], lifetime=0.2, num_segments=20):
    """
    Draw circle using multiple debug lines (most flexible)
    
    Args:
        position: [x, y, z] center position
        radius: circle radius in meters
        color: [r, g, b] color values (0-1)
        lifetime: how long to show (0 = permanent until removed)
        num_segments: number of line segments (higher = smoother circle)
    
    Returns:
        list of debug item IDs for later removal
    """
    debug_ids = []
    
    # Create circle points
    for i in range(num_segments):
        angle1 = 2 * math.pi * i / num_segments
        angle2 = 2 * math.pi * (i + 1) / num_segments
        
        # Calculate points on circle (in XY plane)
        x1 = position[0] + radius * math.cos(angle1)
        y1 = position[1] + radius * math.sin(angle1)
        z1 = position[2]
        
        x2 = position[0] + radius * math.cos(angle2)
        y2 = position[1] + radius * math.sin(angle2)
        z2 = position[2]
        
        # Draw line segment
        line_id = p.addUserDebugLine(
            lineFromXYZ=[x1, y1, z1],
            lineToXYZ=[x2, y2, z2],
            lineColorRGB=color,
            lineWidth=2,
            lifeTime=lifetime
        )
        debug_ids.append(line_id)
    
    return debug_ids

class ProximityWarningSystem:
    def __init__(self, arm_id, gallbladder_id, joint_index=4):
        self.arm_id = arm_id
        self.gallbladder_id = gallbladder_id
        self.joint_index = joint_index
        
        # Warning system variables
        self.warning_active = False
        self.warning_threshold = 0.1  # 10cm safety distance
        self.critical_threshold = 0.05  # 5cm critical distance
        self.warning_id = None
        self.critical_warning_id = None

    def get_joint_position(self):
        """Get the world position of the end effector"""
        link_state = p.getLinkState(self.arm_id, self.joint_index)

        frame_position = link_state[4]  # World position of the link
        frame_orientation = link_state[5]  # Orientation in quaternion

        vector_offset = [0, -0.2, 0.5]  # Offset in local coordinates (e.g., 10cm above the link)

        rotated_vector = Rotation.from_quat(frame_orientation).apply(vector_offset)
        

        return frame_position + rotated_vector # World position of the link
    
    def get_gallbladder_position(self):
        """Get the position of the gallbladder"""
        pos, _ = p.getBasePositionAndOrientation(self.gallbladder_id)
        return pos
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two 3D points"""
        if pos1 is None or pos2 is None:
            return float('inf')
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
    
    def calculate_distance_to_bile_duct(self, pos1):
        """Calculate distance to bile duct (fixed position)"""
        min_dist = float('inf')
        bile_duct_pos_initial = (-0.65, 0.49, 0.077)
        for i in range (14):
            bile_duct_pos = (
                bile_duct_pos_initial[0] - i * 0.1, 
                bile_duct_pos_initial[1] + math.sin(i * 2 * math.pi / 8) * 0.05, 
                bile_duct_pos_initial[2]
            )
            draw_circle(bile_duct_pos, radius=0.1, color=[0, 1, 0])  # Draw gallbladder circle
            distance = self.calculate_distance(pos1, bile_duct_pos)
            if distance < min_dist:
                min_dist = distance
        return min_dist
    
    def check_proximity_and_warn(self):
        """Main function to check proximity and trigger warnings"""
        joint_pos = self.get_joint_position()
        
        if joint_pos is None:
            return
        
        distance = self.calculate_distance_to_bile_duct(joint_pos)
        print("Joint Position:", joint_pos)
        distance_mm = distance * 1000  # Convert to millimeters

        draw_circle(joint_pos)

        print("Distance_mm:", distance_mm)
        
        # Check for critical proximity
        if distance < self.critical_threshold:
            self.trigger_critical_warning(distance_mm)
        elif distance < self.warning_threshold:
            self.trigger_warning(distance_mm)
        else:
            self.clear_warnings()
    
    def trigger_warning(self, distance_mm):
        """Trigger standard warning"""
        if not self.warning_active:
            self.warning_active = True
            
            # GUI warning
            try:
                self.warning_label.config(
                    text=f"WARNING: Too close to gallbladder! ({distance_mm:.1f}mm)",
                    bg='orange'
                )
            except:
                pass
            
            # PyBullet 3D warning
            if self.warning_id is not None:
                p.removeUserDebugItem(self.warning_id)
            
            joint_pos = self.get_joint_position()

            self.warning_id = p.addUserDebugText(
                text=f"WARNING: {distance_mm:.1f}mm",
                textPosition=[joint_pos[0], joint_pos[1], joint_pos[2] + 0.1],
                textColorRGB=[1, 0.5, 0],  # Orange
                textSize=1.5,
                lifeTime=0
            )
            
            print(f"⚠️  WARNING: Joint A is {distance_mm:.1f}mm from gallbladder!")
    
    def trigger_critical_warning(self, distance_mm):
        """Trigger critical warning"""
        # GUI critical warning
        try:
            self.warning_label.config(
                text=f"CRITICAL: Collision risk! ({distance_mm:.1f}mm)",
                bg='red'
            )
        except:
            pass
        
        # PyBullet 3D critical warning
        if self.critical_warning_id is not None:
            p.removeUserDebugItem(self.critical_warning_id)
            
        joint_pos = self.get_joint_position()
        self.critical_warning_id = p.addUserDebugText(
            text=f"CRITICAL: {distance_mm:.1f}mm",
            textPosition=[joint_pos[0], joint_pos[1], joint_pos[2] + 0.15],
            textColorRGB=[1, 0, 0],  # Red
            textSize=2,
            lifeTime=0
        )
        
        print(f"🚨 CRITICAL WARNING: Joint A is only {distance_mm:.1f}mm from gallbladder!")
    
    def clear_warnings(self):
        """Clear all active warnings"""
        if self.warning_active:
            self.warning_active = False
            
            # Clear GUI warning
            try:
                self.warning_label.config(text="Status: SAFE", bg='green')
            except:
                pass
            
            # Clear PyBullet warnings
            if self.warning_id is not None:
                p.removeUserDebugItem(self.warning_id)
                self.warning_id = None
            
            if self.critical_warning_id is not None:
                p.removeUserDebugItem(self.critical_warning_id)
                self.critical_warning_id = None
    
    def toggle_emergency_stop(self):
        """Toggle emergency stop state"""
        pass  # Removed emergency stop functionality
    
    def run_gui_thread(self):
        """Run GUI in separate thread"""
        try:
            self.root.mainloop()
        except:
            pass

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

    # Load gallbladder
    gallbladder_position = [-1.02, -0.1, 0.05]
    gallbladder_orientation = p.getQuaternionFromEuler([0, 0, 1.5])

    gallbladder_id = p.loadURDF(
        "src/urdf/gallbladder.urdf",
        basePosition=gallbladder_position,
        baseOrientation=gallbladder_orientation,
        useFixedBase=True
    )

    # Load the robot arm
    arm_start_pos = [0, 0, 0]
    arm_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    arm_id = p.loadURDF("/src/urdf/onshape.urdf", arm_start_pos, arm_start_orientation, useFixedBase=True)

    # Joint indices
    FIXED_JOINT_IDX = 0
    R_AXIS_JOINT_IDX = 1
    C_AXIS_JOINT_IDX = 2
    B_AXIS_JOINT_IDX = 3
    A_AXIS_JOINT_IDX = 4

    # Initialize proximity warning system for A-axis (joint index 4)
    warning_system = ProximityWarningSystem(arm_id, gallbladder_id, A_AXIS_JOINT_IDX)

    # Infinite simulation loop
    while True:
        # Process queue messages
        while not simulation_queue.empty():
            message = simulation_queue.get()
            # print(f"Simulation: Received Queue Message: {message}")
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

                # Normal movement control - no emergency stop blocking
                p.setJointMotorControl2(
                    arm_id, 
                    joint_index, 
                    p.POSITION_CONTROL, 
                    targetPosition=angle_degrees * math.pi / 180.0  # Convert degrees to radians
                )

        # Check proximity and handle warnings (no automatic stopping)
        warning_system.check_proximity_and_warn()

        p.stepSimulation()
        time.sleep(time_step)