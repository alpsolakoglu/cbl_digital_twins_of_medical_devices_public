import serial
import json

def start(controller_queue, inverse_kinematics_queue, port, baudrate, timeout):
    try:
        # Open the serial port
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")

        # Read data from the serial port
        while True:
            if ser.in_waiting > 0:
                 data = ser.readline().decode('utf-8').strip()
                 print(f"Received: {data}")
            if not controller_queue.empty():
                message = controller_queue.get()
                print(f"Controller: Received Queue Message: {message}")

                if message["type"] == "set_axis_angle":
                    axis_angle_degrees = message["axis_angle_degrees"]
                    axis_name = message["axis_name"]
                    print(f"Controller: Setting {axis_name} axis to {axis_angle_degrees} degrees.")
                    # Here you would send the command to the robot arm via serial
                    # ser.write(f"{axis_name}:{axis_angle_degrees}\n".encode('utf-8'))

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

