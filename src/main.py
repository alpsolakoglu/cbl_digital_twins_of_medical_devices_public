import threading
import multiprocessing as mp
import queue
import controller
import simulation as sim

def main():  
    # Create shared queues
    simulation_queue = mp.Queue()  # For sending data to simulation
    controller_queue = mp.Queue()  # For sending data to controller

    controller_process = mp.Process(target=controller.start, args=(controller_queue, simulation_queue))
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