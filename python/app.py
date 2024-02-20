from Models.Drone import Drone
from CLIFunctions.cliFunctions import addDroneToSimulation, issueStartCommand, startEmptyGazeboWorldSimulation, destroyDrone
from random import randint
from threading import Thread
from icecream import ic
from time import time, sleep
from subprocess import Popen

START_FLAG = 0
COUNTER = 0
SYSTEM_RUNNING = False
LEAKER_COUNT = 0

# # from fakeSimulationFunctions import simulateAddingDrone, writeDictToCSV
# def initializeDrone(droneID:int,droneDirectory:dict, locationDirectory:dict) -> None:
#         startingLocation = spawnDroneRandomCoordinates(droneID=droneID)
#         droneDirectory[droneID]=(Drone(droneID=droneID, startingLocation=startingLocation, currentLocation=startingLocation))
#         locationDirectory[droneID]=(startingLocation)


"""

Constantly reads the stdout of the provided subprocess, waiting for the target line to appear.
options: 
        0 - Does not print lines (Defualt)    
        1 - Print Lines While Waiting (Used for monitoring world creation)
    
"""
def waitForLineInSubprocess(subprocess:Popen, targetStr:str, option:int = 0):
    line = subprocess.stdout.readline()
    while(not line.startswith(targetStr)):
        if option == 1:
            ic(line) # prints current line to verify what process is doing
        line = subprocess.stdout.readline()
    return


def delayedStartCommand(numberOfDrones:int) -> None:
    global COUNTER, START_FLAG
    
    # while waiting for all the subprocesses to be created, the delayedStartCommand does nothing
    while COUNTER != numberOfDrones:
        continue
    # # Once all of the drones have been spawned in, the function waits 10 seconds (arbitrary), and issues the start command, causing all drones to travel to the center
    sleep(10)
    issueStartCommand()
    # the start flag is sent, allowing all subprocesses to terminate and kill themselves
    START_FLAG = 1
    return

"""
DOES WAY TOO MUCH, however this creates random spawn coordinates, then starts the subprocess which contains the robot/drone's "brain"
waits until robot/drone has reached (0,0) +/- 0.5m and then increments the leaker count, updates the drone's class model, and removes the drone from the sim
"""
def droneController(droneID:int, droneDirectory:dict, subprocesses:dict) -> None:
    global COUNTER, LEAKER_COUNT
    randomRange = (10,20)
    startingX = randint(a=randomRange[0],b=randomRange[1])
    startingY = randint(a=randomRange[0],b=randomRange[1])
    startingZ = 0
    quadrant = (randint(1,4))
    if quadrant == 1:
        startingX *= 1 
        startingY *= 1 
    elif quadrant == 2:
        startingX *= -1 
        startingY *= 1 
    elif quadrant == 3:
        startingX *= -1 
        startingY *= -1 
    elif quadrant == 4:
        startingX *= 1 
        startingY *= -1 
        
    
    print(f"Spawning Drone at ({startingX},{startingY},{startingZ})")
    # calls the function which creates and returns a subprocess object which executes the "add_robots_to_simulation" command
    droneSubprocess = addDroneToSimulation(droneID=droneID, spawnCoordinateX=startingX, spawnCoordinateY=startingY)
    # once the subprocess is successfully created, the global counter is incremented and the drone's information is stored
    waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[INFO] [add_bot_node-1]: process has finished cleanly [pid")
    COUNTER += 1
    # Starting Location  = [x,y,z]
    tempDrone = Drone(droneID=droneID, 
                      currentStatus= "Alive",
                      startingLocation=[startingX,startingY,startingZ],
                      currentLocation=[startingX,startingY,startingZ])
    droneDirectory[droneID] = tempDrone
    subprocesses[f"robot_{droneID}"] = droneSubprocess
    
    # wait for signal that drone has reached the origin, then begin the process of deleting robot
    waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=0)
    droneDirectory[droneID].currentStatus="Leaker"
    LEAKER_COUNT += 1
    destroyDrone(droneID=droneID)
    subprocesses.pop(f"robot_{droneID}")
    
    
    # # Keep subprocess running for as long as the program is, so that the robots keep their movement patterns
    # while SYSTEM_RUNNING:        
    #     continue
    
    # while waiting for the start command to signal the subprocesses can be killed, there is a debugging message
    # while START_FLAG != 1:
    #     continue
    # # after pausing, the subprocess is terminated and killed
    
    droneSubprocess.terminate()
    sleep(3)
    droneSubprocess.kill()
    ic(f"Drone {droneID} Process Killed")
    
    return 
    
    
def getUserInput(prompt:str, expectedType):
    if type(response:=expectedType(input(prompt))) is not expectedType:
        print("Enter an Integer")
        getUserInput(prompt=prompt, expectedType=expectedType)
    else:
        return response

def main():
    global SYSTEM_RUNNING
    droneDirectory, locationDirectory, subprocesses = {}, {}, {}
    


    print("                                Welcome to the BOWSER Prototype ---- DEMO ---- ")
    
    
    # numberOfDrones = getUserInput(prompt="""
    #                        Please enter a number of drones to simulate ( Max: 15 )
    #                        """, expectedType=int)
        
    # algorithmChoice = getUserInput("""
    #                         Please Input 1 - 4 to Select Algorithm:
    #                         ---------------------------------------
    #                         1: Deep Q Network
    #                         2: Genetic Algorithm
    #                         3: Munkres (Hungarian)
    #                         4: Simulated Annealing
    #                         """, expectedType=int)+1

    numberOfDrones, algorithmChoice = 1,2
    
    print(f"You have selected {numberOfDrones} Drones")
    ic(algorithmChoice)

    ic("Start Empty Gazebo World")
    """ ? Use subprocess.Popen ? """
    worldCreationProcess = startEmptyGazeboWorldSimulation(totalNumberOfDrones=2)
    waitForLineInSubprocess(subprocess=worldCreationProcess, targetStr="[INFO] [ground_truth_publisher-2]: process started with pid", option=1)
    ic("World Created")    
    SYSTEM_RUNNING = True
    sleep(5)
    
    print(f"Spawn in {numberOfDrones} Drones")
    threads = []
    for x in range(numberOfDrones):
        # Create a thread for each drone
        thread = Thread(target=droneController, args=(x,droneDirectory, subprocesses))
        thread.start()
        threads.append(thread)
    # Once each drone thread has been created, A final "timer" thread is started which monitors the number of threads that have been executed
    # Once all of the threads have begun executing, this thread will wait a moment and then send the start command, which will then prompt the drone
    # threads to terminal and kill the subprocesses, allowing the algorithms and such to begin
    startDroneThread = Thread(target=delayedStartCommand,args=(numberOfDrones,))
    startDroneThread.start()
    threads.append(startDroneThread)

    # Wait for threads to finish before continuing 
    while START_FLAG != 1:
        continue
    
    ic(droneDirectory)
    ic(subprocesses)
    lc = LEAKER_COUNT
    while len(subprocesses) != 0:
        if lc != LEAKER_COUNT:
            ic(LEAKER_COUNT)
            lc = LEAKER_COUNT
        continue
    ic("No more Subprocesses, all robots should be dead")
    # ic(droneDirectory)
    
    
    # ic("Writing Drone Locations into CSV for Algorithms")
    
    # writeDictToCSV(locationDirectory,"simulatedDroneLocations.csv")
    
    
    # ic("Call Algorithm - Currently Using Template Threat Locations For Sake of Presentation")
    # startTime = time()
    # # If they chose to run DQN, first it trains a model, then runs it, requiring two algorithm calls
    # if algorithmChoice == 2:
    #     selectAlgorithm(algorithmChoice=algorithmChoice-1)
    # results = selectAlgorithm(algorithmChoice=algorithmChoice)
    # finishTime = time()
    
    # ic("Handle Drones in Simulation According to Algorithm Results") 
    
    # ic("Calculate Simulation Leaker Percentage")
    # # ic(results)
    # print("Returned Output")
    # ic(results["Weapon Selection"])
    # print(" \n\n            -------------------------------------------------------------------------------------------------")
    # print(f"            Simulated Leaker Percentage: XX.XX% | Algorithm Percentage: {results['Leaker Percentage']}%\n\n\n")
    # print(f"                                      That Took {finishTime-startTime} Seconds")

if __name__ == "__main__":
    
    main()
    # print("s")


