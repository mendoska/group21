from Models.Drone import Drone
from CLIFunctions.cliFunctions import addDroneToSimulation, issueStartCommand
from time import sleep
from icecream import ic
from threading import Thread



START_FLAG = 0
    
def delayedStartCommand(numberOfDrones:int) -> None:
    global counter
    global START_FLAG
    # while waiting for all the subprocesses to be created, the delayedStartCommand does nothing
    while counter != numberOfDrones:
        continue
    # # Once all of the drones have been spawned in, the function waits 10 seconds (arbitrary), and issues the start command, causing all drones to travel to the center
    sleep(10)
    issueStartCommand()
    # the start flag is sent, allowing all subprocesses to terminate and kill themselves
    START_FLAG = 1
    return
    

def startSpawnProcess(droneID:int) -> None:
    # just varying the spawn location so they dont spawn ontop of one another
    xC = int(droneID) * 5
    yC = droneID * -4 if droneID % 2 == 0 else 4
    global counter
    # calls the function which creates and returns a subprocess object which executes the "add_robots_to_simulation" command
    subprocess = addDroneToSimulation(droneID=droneID, spawnCoordinateX=xC ,spawnCoordinateY=yC)
    # once the subprocess is successfully created, the global counter is incremented and the drone's information is stored
    counter += 1
    tempDrone = Drone(droneID=droneID)
    droneDirectory[droneID]=(tempDrone)
    # while waiting for the start command to signal the subprocesses can be killed, there is a debugging message
    while START_FLAG != 1:
        # ic(f"{droneID} Awaiting Start Command")
        # sleep(3)
        continue
    # after pausing, the subprocess is terminated and killed
    sleep(5)
    subprocess.terminate()
    sleep(3)
    subprocess.kill()
    return

def main():
    summonNumberOfDrones = 5  # Set your desired number of drones here
    threads = []
    
    for x in range(summonNumberOfDrones):
        # Create a thread for each drone
        thread = Thread(target=startSpawnProcess, args=(x,))
        thread.start()
        threads.append(thread)
    # Once each drone thread has been created, A final "timer" thread is started which monitors the number of threads that have been executed
    # Once all of the threads have begun executing, this thread will wait a moment and then send the start command, which will then prompt the drone
    # threads to terminal and kill the subprocesses, allowing the algorithms and such to begin
    startDroneThread = Thread(target=delayedStartCommand,args=(summonNumberOfDrones,))
    startDroneThread.start()
    threads.append(startDroneThread)

    # Wait for all threads to complete
    for thread in threads:
        thread.join()
        

    print("All threads completed.")

if __name__ == "__main__":
    global counter
    counter = 0
    droneDirectory = {}
    main()
    ic(droneDirectory)
    ic(counter)






