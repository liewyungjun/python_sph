from multiprocessing import Process
import os
import time

def square_numbers():
    for i in range(100):
        j = i+1
        result = i * i
        time.sleep(0.0000000001)


if __name__ == "__main__":        
    processes = []
    num_processes = os.cpu_count()
    multiprocessing = True
    # number of CPUs on the machine. Usually a good choise for the number of processes
    start_time = time.time()
    for i in range(num_processes):
            process = Process(target=square_numbers)
            processes.append(process)
    if multiprocessing:
        # create processes and asign a function for each process
        
        # start all processes
        for process in processes:
            process.start()

        # wait for all processes to finish
        # block the main programm until these processes are finished
        for process in processes:
            process.join()
    else:
        for i in range(num_processes):
            square_numbers()
        
    end_time = time.time()
    print(f'Time taken: {end_time - start_time} seconds')
    
