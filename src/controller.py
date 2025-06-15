import serial
import json
import math


axis_names_ordered = ["R", "A", "B", "C"]  # Ordered list of axis names
def start(controller_queue, inverse_kinematics_queue, port, baudrate, timeout):
    try:
        # Open the serial port
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")

        # Read data from the serial port
        while True:
            # if ser.in_waiting > 0:
            #      data = ser.readline().decode('utf-8').strip()
            #      print(f"Received: {data}")
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

