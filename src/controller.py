import serial
import json
import math
import re
import time

def parse_robot_angle(data):
    """
    Parse individual RobotAngle strings like 'RobotAngleA:45.67'
    Returns tuple of (axis_name, angle_degrees) or None if parsing fails
    """
    # Use regex to match the pattern RobotAngle[Letter]:[Number]
    match = re.match(r'RobotAngle([RABC]):(-?\d+\.?\d*)', data)
    if match:
        axis_name = match.group(1)
        angle_degrees = float(match.group(2))
        return axis_name, angle_degrees
    return None


axis_names_ordered = ["R", "A", "B", "C"]  # Ordered list of axis names
def start(controller_queue, simulation_queue, port, baudrate, timeout):
    try:
        # Open the serial port
        while True:
            time.sleep(3)
            try:
                print(f"Attempting to connect to {port} at {baudrate} baud...")
                ser = serial.Serial(port, baudrate, timeout=timeout)
                break
            except serial.SerialException as e:
                print(f"Failed to connect to {port} at {baudrate} baud: {e}")
                continue

        print(f"Connected to {port} at {baudrate} baud.")

        # Read data from the serial port
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()

                # Try to parse as individual robot angle
                if data.startswith("RobotAngle"):
                    parsed = parse_robot_angle(data)
                    if parsed:
                        axis_name, angle_degrees = parsed
                        
                        simulation_queue.put({
                            "type": "twin_axis_angle",
                            "axis_angle_degrees": angle_degrees,
                            "axis_name": axis_name
                        })
                    else:
                        print(f"Failed to parse robot angle data: {data}")

            if not controller_queue.empty():
                message = controller_queue.get()
                print(f"Controller: Received Queue Message: {message}")

                if message["type"] == "set_all_angles":
                    joint_angles_rad = message["angles_rad"]
                    print("Controller: Received new IK solution. Setting all joint angles.")
                    if len(joint_angles_rad) == len(axis_names_ordered):
                        for i, angle_rad in enumerate(joint_angles_rad):
                            axis_name = axis_names_ordered[i]
                            angle_deg = math.degrees(angle_rad)
                            print(f"Controller: Setting {axis_name} axis to {angle_deg:.2f} degrees.")
                            # Here you would format and send the command for each joint
                            # ser.write(f"{axis_name}:{angle_deg}\n".encode('utf-8'))
                    else:
                        print("Controller: Error - Mismatch between received angles and expected number of axes.")

                elif message["type"] == "stop":
                    print("Controller: Stopping process.")
                    break
                # Process the message as needed

    #except serial.SerialException as e:
    #    print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    # finally:
    #     if ser.is_open:
    #         ser.close()
    #         print("Serial port closed.")

