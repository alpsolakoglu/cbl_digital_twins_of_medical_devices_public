import threading
import multiprocessing as mp
import queue
import controller
import simulation as sim

def main():  
    # Create shared queues
    simulation_queue = mp.Queue()  # For sending data to simulation
    controller_queue = mp.Queue()  # For sending data to controller

    simulation_queue.put({"type": "set_axis_angle", "axis_angle_degrees": 90, "axis_name": "B"})  # Example input for simulation
    simulation_queue.put({"type": "set_axis_angle", "axis_angle_degrees": 0, "axis_name": "B"})  # Example input for simulation
    simulation_queue.put({"type": "set_axis_angle", "axis_angle_degrees": 90, "axis_name": "B"})  # Example input for simulation
    simulation_queue.put({"type": "set_axis_angle", "axis_angle_degrees": 0, "axis_name": "B"})  # Example input for simulation

    simulation_queue.put({"type": "stop"})  # Example input for simulation
    controller_queue.put({"type": "stop"})  # Example input for simulation

    controller_process = mp.Process(target=controller.start, args=(controller_queue, simulation_queue, "COM", 115200, 1))
    simulation_process = mp.Process(target=sim.start, args=(controller_queue, simulation_queue))

    print("Starting processes...")

    # Start both processes
    controller_process.start()
    simulation_process.start()
    
    # Wait for both processes to finish
    controller_process.join()
    simulation_process.join()

    print("All processes completed!")
if __name__ == "__main__":
    main()