from Models.Drone import Drone
from Models.Weapon import Weapon
from CLIFunctions.cliFunctions import addDroneToSimulation, issueStartCommand, startEmptyGazeboWorldSimulation, destroyDrone
from random import randint, uniform
from threading import Thread, Lock, Condition
from icecream import ic
from time import time, sleep
from subprocess import Popen
from csv import DictWriter, reader
from math import cos, sin, pi
from yaml import safe_load, dump

START_FLAG = 0
SYSTEM_RUNNING = False
START_COUNTER = 0
LEAKER_COUNT = 0

# global thread lock
file_lock = Lock()

# Define a condition variable
file_condition = Condition(file_lock)

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
    
    Loads Defense Weaponry from File into a list of Weapon objects, allowing for simple handling
    Intended for multiple placements throughout the area in future
    
    """


def initializeWeaponsFromFile(threat_file)-> list:
    with open(threat_file, 'r') as file:
        weapons = []
        readerObj = reader(file)
        for weaponID, threatEntry in enumerate(list(readerObj)):
            weaponPlacement = Weapon(weaponID=weaponID,
                                     weaponName=threatEntry[0],
                                     location=[threatEntry[1], threatEntry[2], threatEntry[3]],
                                     ammunitionQuantity=threatEntry[5],
                                     firingRate=threatEntry[6]
                                    )
            weapons.append(weaponPlacement)
        
        return weapons


def startWeaponSystem(weaponModel:Weapon, targetList:list, placement:set, droneDirectory:dict) -> None:
    global LEAKER_COUNT
    for targetID in targetList:
        targetThreat = droneDirectory[targetID]
        if targetThreat.currentStatus == "Alive":
            if weaponModel.ammunitionQuantity <= 0:
                ic(f"{weaponModel.weaponName} has run out of ammunition, Drone {targetID} as become a leaker")
                targetThreat.currentStatus = "Leaker"
                LEAKER_COUNT += 1
                
            ic(f"{weaponModel.weaponName} has fired once at Drone {targetID}")
            weaponModel.ammunitionQuantity -= 1
            deletionAttempt = destroyDrone(droneID=targetID)
            if deletionAttempt.startswith("Successfully deleted entity"):
                targetThreat.currentStatus="Dead"
            
            ic(f"Drone {targetID} Destroyed With {weaponModel.weaponName}")


# spawnParameterLock = Lock()

def alterD2OSpawnParameters(s:float = 1.0, lr:float = 1.0, droneID=int ) -> None:
    
    filePath = '../simulation/swarm_ws/ROS2swarm_B/install/ros2swarm/share/ros2swarm/config/waffle_pi/movement_pattern/basic/drive2OriginPattern.yaml'
    with open(file=filePath,mode='r') as file:
        data = safe_load(file)
    # ic(data["/**"]["ros__parameters"])
    data["/**"]["ros__parameters"]["speed"] = s
    data["/**"]["ros__parameters"]["leaker_range"] = lr
    ic(droneID,s, lr,data["/**"]["ros__parameters"])
    # Write back to YAML file
    with open(file=filePath, mode='w') as file:
        dump(data, file)

"""

Constantly reads the stdout of the provided subprocess, waiting for the target line to appear.
options: 
        0 - Does not print lines (Defualt)    
        1 - Print Lines While Waiting (Used for monitoring world creation)
    
"""
def waitForLineInSubprocess(subprocess:Popen, targetStr:str, option:int = 0):
    line = subprocess.stdout.readline()
    while(not targetStr in line):
        if option == 1:
            ic(line) # prints current line to verify what process is doing
        line = subprocess.stdout.readline()
    print(line)
    return


def delayedStartCommand(numberOfDrones:int) -> None:
    global START_COUNTER, START_FLAG
    
    # while waiting for all the subprocesses to be created, the delayedStartCommand does nothing
    while START_COUNTER != numberOfDrones:
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
def droneController(droneID:int, droneDirectory:dict, subprocesses:dict, spawnRange:set, locationDict:dict, spawnParams:set, threatCoordinates=None) -> None:
    global START_COUNTER, LEAKER_COUNT

    if threatCoordinates and droneID+1 in threatCoordinates:
        # Use user-defined coordinates
        startingX = threatCoordinates[droneID+1]["x"]
        startingY = threatCoordinates[droneID+1]["y"]
        # startingZ = threatCoordinates[droneID+1]["z"]
        startingZ = 0
    else:
        """ polar to cartesian conversion from ChatGPT, just didnt want to do math """
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


    with file_lock:
        print(f"Spawning Drone at ({startingX},{startingY},{startingZ}), Speed: {spawnParams[0]}, Leaker Range: {spawnParams[1]}")
        alterD2OSpawnParameters(s=spawnParams[0], lr=spawnParams[1], droneID=droneID)

        # calls the function which creates and returns a subprocess object which executes the "add_robots_to_simulation" command
        droneSubprocess = addDroneToSimulation(droneID=droneID, spawnCoordinateX=startingX, spawnCoordinateY=startingY)
        # waitForLineInSubprocess(subprocess=droneSubprocess, targetStr=f"Leaker Range is: {spawnParams[1]}", option=1)
        # once the subprocess is successfully created, the global counter is incremented and the drone's information is stored
        locationDict[droneID] = {"x":startingX,"y":startingY,"z":startingZ, "minRange": spawnRange[0]}    
        waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[INFO] [add_bot_node-1]: process has finished cleanly [pid", option=0)
        file_condition.notify_all()
    START_COUNTER += 1
        
        
    # Starting Location  = [x,y,z]
    tempDrone = Drone(droneID=droneID, 
                      currentStatus= "Alive",
                      startingLocation=[startingX,startingY,startingZ],
                      currentLocation=[startingX,startingY,startingZ])
    droneDirectory[droneID] = tempDrone
    subprocesses[f"robot_{droneID}"] = droneSubprocess
    
    # wait for signal that drone has reached the origin, then begin the process of deleting robot (Checks twice because false flags have appeared)
    waitForLineInSubprocess(subprocess=droneSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=1)
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
    

def run_BOWSER_simulation(spawnRange:set, algorithmChoice:str, numberOfDrones:int, threatCoordinates=None) -> tuple:
    global SYSTEM_RUNNING
    
    threatFileLocation = "dataFiles/simulationDroneLocations.csv"
    weaponFileLocation = "dataFiles/weapon_data.csv"
    dqnModelPath = "dataFiles/trained_model.zip"
    
    
    droneDirectory, locationDirectory, subprocesses = {}, {}, {}
    


    print("                                Welcome to the BOWSER Prototype ---- DEMO ---- ")
   
    print(f"You have selected {numberOfDrones} Drones")
    ic(algorithmChoice)
    
    ic("Start Empty Gazebo World")
    worldCreationProcess = startEmptyGazeboWorldSimulation(totalNumberOfDrones=2)
    waitForLineInSubprocess(subprocess=worldCreationProcess, targetStr="[INFO] [ground_truth_publisher-2]: process started with pid", option=1)
    ic("World Created")    
    weaponList = initializeWeaponsFromFile(threat_file=weaponFileLocation)
    SYSTEM_RUNNING = True
    
    print(f"Spawn in {numberOfDrones} Drones")
    threads = []
    for x in range(numberOfDrones):
        # Create a thread for each drone
        thread = Thread(target=droneController, args=(x,droneDirectory, subprocesses, spawnRange, locationDirectory, (float(x+1), float(x+1)), threatCoordinates))
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
    
    """
    Assigning Targets to defense weaponry, right now assuming there are 1 of each weapon system, all located at (0,0)
    """
    # [[short range targets],[medium Range Targets], [Long Range Targets], [Directed Energy Targets]] 
    targetDelegations = [[],[],[],[]]
    for target in response:
        # if assigned weapon is short range, add to the short range targets 
        if target[1] == weaponList[0].weaponName:
            targetDelegations[0].append(int(target[0])) 
        # if assigned weapon is medium range, add to the medium range targets 
        elif target[1] == weaponList[1].weaponName:
            targetDelegations[1].append(int(target[0])) 
        # if assigned weapon is long range, add to the long range targets 
        elif target[1] == weaponList[2].weaponName:
            targetDelegations[2].append(int(target[0])) 
        # if none else, it should be a directed energy target, thus add to the D.E targets 
        else:
            targetDelegations[3].append(int(target[0])) 
      
    weaponThreads = []  
    for weaponIndex, weaponSystem in enumerate(weaponList):
        weaponThread = Thread(target=startWeaponSystem, args=(weaponSystem, [weaponIndex],(0,0,0), droneDirectory))
        weaponThread.start()
        weaponThreads.append(weaponThread)
        
    for thread in weaponThreads:
        thread.join()
        
        # ic(weaponFiring)
        # droneID = int(weaponFiring[0])
        # weapon = weaponFiring[1]
        # target = droneDirectory[droneID]
        # if target.currentStatus == "Alive":
        #     deletionAttempt = destroyDrone(droneID=droneID)
        #     if deletionAttempt.startswith("Successfully deleted entity"):
        #         target.currentStatus="Dead"
            
        #     ic(f"Drone {droneID} Destroyed With {weapon}")
    
    
    
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
    
    run_BOWSER_simulation(spawnRange=(10,20), algorithmChoice="DQN", numberOfDrones=3)
    # print("s")


