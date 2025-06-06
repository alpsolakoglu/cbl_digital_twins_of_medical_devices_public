import threading
import subprocess
import sys
import queue

def run_intermediary_controller(file_location="./src/intermediary_controller.py"):
    try:
        print(f"Starting {file_location}...")
        result = subprocess.run([sys.executable, file_location], capture_output=True, text=True)
        
        
        if result.stderr:
            print(f"{file_location} errors:")
            print(result.stderr)

        print(f"{file_location} finished with return code: {result.returncode}")
    except FileNotFoundError:
        print(f"Error: {file_location} not found in current directory")
    except Exception as e:
        print(f"Error running second process: {e}")

def run_inverse_kinematics(file_location="./src/inverse_kinematics.py"):
    try:
        print(f"Starting {file_location}...")
        result = subprocess.run([sys.executable, file_location], capture_output=True, text=True)
        

        if result.stderr:
            print(f"{file_location} errors:")
            print(result.stderr)   
        print(f"{file_location} finished with return code: {result.returncode}")
    except FileNotFoundError:
        print(f"Error: {file_location} not found in current directory")
    except Exception as e:
        print(f"Error running {file_location}: {e}")

def run_file(file_location):
    try:
        print(f"Starting {file_location}...")
        result = subprocess.run([sys.executable, file_location], capture_output=True, text=True)
        

        if result.stderr:
            print(f"{file_location} errors:")
            print(result.stderr)   
        print(f"{file_location} finished with return code: {result.returncode}")
    except FileNotFoundError:
        print(f"Error: {file_location} not found in current directory")
    except Exception as e:
        print(f"Error running {file_location}: {e}")

def main():  
    # Create shared queues
    inverse_kinematics_queue = queue.Queue()  # For sending data to inverse_kinematics thread
    intermediary_controller_queue = queue.Queue()  # For sending data to intermediary_controller thread
    
    # Put some example input data in the queues
    #inverse_kinematics_queue.put({"joint_angle_degrees": 15, "axis_letter": "A"})
    #intermediary_controller_queue.put(42) 
    
    # Create threads for each process
    thread1 = threading.Thread(target=run_intermediary_controller, 
                              args=(intermediary_controller_queue, inverse_kinematics_queue),
                              name="intermediary_controller")
    thread2 = threading.Thread(target=run_inverse_kinematics, 
                              args=(inverse_kinematics_queue, intermediary_controller_queue),
                              name="inverse_kinematics")

    # Start both threads
    thread1.start()
    thread2.start()
    
    print("Both processes started in separate threads...")
    
    # Wait for both threads to complete
    thread1.join()
    thread2.join()
    
    print("All processes completed!")

if __name__ == "__main__":
    main()