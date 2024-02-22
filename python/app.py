from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing

from Models.Drone import Drone
from CLIFunctions.cliFunctions import addDroneToSimulation, issueStartCommand, startEmptyGazeboWorldSimulation, destroyDrone
from random import randint, uniform
from threading import Thread
from icecream import ic
from time import time, sleep
from subprocess import Popen
from csv import DictWriter
from math import cos, sin, pi

START_FLAG = 0
COUNTER = 0
SYSTEM_RUNNING = False
LEAKER_COUNT = 0

# # from fakeSimulationFunctions import simulateAddingDrone, writeDictToCSV
# def initializeDrone(droneID:int,droneDirectory:dict, locationDirectory:dict) -> None:
#         startingLocation = spawnDroneRandomCoordinates(droneID=droneID)
#         droneDirectory[droneID]=(Drone(droneID=droneID, startingLocation=startingLocation, currentLocation=startingLocation))
#         locationDirectory[droneID]=(startingLocation)


def writeDictToCSV(dictionary, csv_filename):
    """
    Write a Python dictionary to a CSV file.

    Parameters:
        dictionary (dict): The dictionary to be written to the CSV file.
        csv_filename (str): The name of the CSV file.

    Returns:
        None
    """
    with open(csv_filename, 'w', newline='') as csvfile:
        fieldnames = ['droneID', 'x', 'y', 'z', 'minRange', 'Speed', 'Type' ]
        writer = DictWriter(csvfile, fieldnames=fieldnames)

        # Write the header
        # writer.writeheader()

        # Write each key-value pair as a separate row
        for key, values in dictionary.items():
            writer.writerow({'droneID': key, 'x': values.get('x', ''), 'y': values.get('y', ''), 'z': values.get('z', ''), 
                             'minRange':values.get('minRange',''), 'Speed': 1, 'Type': 'A'})


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
    print(line)
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
def droneController(droneID:int, droneDirectory:dict, subprocesses:dict, spawnRange:set, locationDict:dict) -> None:
    global COUNTER, LEAKER_COUNT
    
    
    """ lines 92 - 103 from ChatGPT, just didnt want to do math """
    if spawnRange[0] > spawnRange[1]:
        raise ValueError("Minimum radius cannot be greater than maximum radius")

    # Generate random angle within 0 to 2*pi
    angle = uniform(0, 2 * pi)
    
    # Generate random radius within the range [min_radius, max_radius]
    radius = uniform(spawnRange[0], spawnRange[1])
    
    # Calculate x and y coordinates using polar coordinates to Cartesian coordinates conversion
    startingX = radius * cos(angle)
    startingY = radius * sin(angle)
    startingZ = 0

    
    print(f"Spawning Drone at ({startingX},{startingY},{startingZ})")
    # calls the function which creates and returns a subprocess object which executes the "add_robots_to_simulation" command
    droneSubprocess = addDroneToSimulation(droneID=droneID, spawnCoordinateX=startingX, spawnCoordinateY=startingY)
    # once the subprocess is successfully created, the global counter is incremented and the drone's information is stored
    locationDict[droneID] = {"x":startingX,"y":startingY,"z":startingZ, "minRange": spawnRange[0]}
    waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[INFO] [add_bot_node-1]: process has finished cleanly [pid")
    COUNTER += 1
    # Starting Location  = [x,y,z]
    tempDrone = Drone(droneID=droneID, 
                      currentStatus= "Alive",
                      startingLocation=[startingX,startingY,startingZ],
                      currentLocation=[startingX,startingY,startingZ])
    droneDirectory[droneID] = tempDrone
    subprocesses[f"robot_{droneID}"] = droneSubprocess
    
    # wait for signal that drone has reached the origin, then begin the process of deleting robot (Checks twice because false flags have appeared)
    waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=0)
    waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=0)
    droneDirectory[droneID].currentStatus="Leaker"
    LEAKER_COUNT += 1
    destroyDrone(droneID=droneID)
    subprocesses.pop(f"robot_{droneID}")
    
    # Cleaning up subprocess, (terminate and kill it) 
    droneSubprocess.terminate()
    sleep(3)
    droneSubprocess.kill()
    ic(f"Drone {droneID} Process Killed")
    
    return 
    
""" Used For Testing, Not Needed In Final Demo """
def getUserInput(prompt:str, expectedType):
    if type(response:=expectedType(input(prompt))) is not expectedType:
        print("Enter an Integer")
        getUserInput(prompt=prompt, expectedType=expectedType)
    else:
        return response

def run_BOWSER_simulation(spawnRange: set, algorithmChoice: str, numberOfDrones:int) -> tuple:
    global SYSTEM_RUNNING
    
    threatFileLocation = "dataFiles/simulationDroneLocations.csv"
    weaponFileLocation = "dataFiles/weapon_data.csv"
    dqnModelPath = "dataFiles/trained_model.zip"
    
    
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
        thread = Thread(target=droneController, args=(x,droneDirectory, subprocesses, spawnRange, locationDirectory))
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
    
    
    """Write Drone Locations into CSV for Algorithms"""
    
    writeDictToCSV(locationDirectory,threatFileLocation)
    
    "" "Call Algorithm """    
    start = time()
    if algorithmChoice == "DQN":
        from algorithms.dqn_agent import runDQN
        runDQN(savePath=dqnModelPath, train=True, num_threats=numberOfDrones)
        sleep(3)
        response, algorithm_leaker_percentage = runDQN(loadPath=dqnModelPath, train=False, threatFilePath=threatFileLocation)
    elif algorithmChoice == "Genetic Algorithm":
        from algorithms.geneticAlgorithmTest import runGA
        response, algorithm_leaker_percentage = runGA(threatFileLocation=threatFileLocation)
        algorithm_leaker_percentage = (1.00 - algorithm_leaker_percentage) * 100

    elif algorithmChoice == "Munkres":
        from algorithms.munkres_algorithm import runMunkres
        response, algorithm_leaker_percentage = runMunkres(threatFileLocation=threatFileLocation, weaponFileLocation=weaponFileLocation)

    elif algorithmChoice == "Simulated Annealing":
        from algorithms.simulated_annealing import runSimulatedAnnealing
        response, algorithm_leaker_percentage = runSimulatedAnnealing()
        leaker_percealgorithm_leaker_percentagenalgorithm_leaker_percentagetage *= 100
    else:
        raise "Invalid Algorithm Choice"
    
    end = time()
    
    """Handle Drones in Simulation According to Algorithm Results"""
    ic(droneDirectory)
    ic(subprocesses)
    
    print(f"Waiting {(end-start)} seconds to offset Sim Time to Real Time difference")
    sleep(end-start)
    for weaponFiring in response:
        ic(droneDirectory)
        ic(weaponFiring)
        droneID = int(weaponFiring[0])
        weapon = weaponFiring[1]
        target = droneDirectory[droneID]
        if target.currentStatus == "Alive":
            deletionAttempt = destroyDrone(droneID=droneID)
            if deletionAttempt.startswith("Successfully deleted entity"):
                target.currentStatus="Dead"
            
            ic(f"Drone {droneID} Destroyed With {weapon}")
    
    
    
    try:
        for subprocessID in subprocesses:
            # ic(subprocesses)
            subprocesses[subprocessID].kill()
    except: 
        pass
     
    # ic(droneDirectory)
    # ic(subprocesses)
    lc = LEAKER_COUNT
    while LEAKER_COUNT + len(subprocesses) != numberOfDrones:
        if lc != LEAKER_COUNT:
            ic(LEAKER_COUNT)
            lc = LEAKER_COUNT
        continue
    # Clean Up and remaining subprocesses
    ic("No more Subprocesses, all robots should be dead")

    simulation_leaker_percent = (LEAKER_COUNT / numberOfDrones) * 100 
    ic(simulation_leaker_percent, algorithm_leaker_percentage)
    # ic("Calculate Simulation Leaker Percentage")
    # # ic(results)
    # print("Returned Output")
    # ic(results["Weapon Selection"])
    # print(" \n\n            -------------------------------------------------------------------------------------------------")
    # print(f"            Simulated Leaker Percentage: XX.XX% | Algorithm Percentage: {results['Leaker Percentage']}%\n\n\n")
    # print(f"                                      That Took {finishTime-startTime} Seconds")

    return simulation_leaker_percent, algorithm_leaker_percentage


if __name__ == "__main__":
    
    run_BOWSER_simulation(spawnRange=(10,20), algorithmChoice="DQN", numberOfDrones=10)
    # print("s")


