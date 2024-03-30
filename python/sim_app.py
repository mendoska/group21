from Models.Drone import Drone
from Models.Weapon import Weapon
from random import randint, uniform
from icecream import ic
from threading import Thread
from time import time, sleep
from csv import DictWriter, reader
from math import cos, sin, pi
from yaml import safe_load, dump


LEAKER_COUNT = 0


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


def initializeWeaponsFromFile(weapon_file:str)-> list:
    with open(weapon_file, 'r') as file:
        weapons = []
        readerObj = reader(file)
        # ic(list(readerObj))
        for weaponID, threatEntry in enumerate(list(readerObj)):
            # ic(threatEntry)
            weaponPlacement = Weapon(weaponID=weaponID,
                                     weaponName=threatEntry[0],
                                     location=[threatEntry[1], threatEntry[2], threatEntry[3]],
                                     ammunitionQuantity=int(threatEntry[5]),
                                     firingRate=threatEntry[6]
                                    )
            weapons.append(weaponPlacement)
        
        return weapons


def startWeaponSystem(weaponModel:Weapon, targetList:list, placement:set, droneDirectory:dict) -> None:
    global LEAKER_COUNT
    ic(targetList)
    for targetID in targetList:
        
        targetThreat = droneDirectory[targetID]
        if targetThreat.currentStatus == "Alive":
            if weaponModel.ammunitionQuantity <= 0:
                ic(f"{weaponModel.weaponName} has run out of ammunition, Drone {targetID} as become a leaker")
                targetThreat.currentStatus = "Leaker"
                LEAKER_COUNT += 1
                
            ic(f"{weaponModel.weaponName} has fired once at Drone {targetID}")
            weaponModel.ammunitionQuantity -= 1
            # deletionAttempt = destroyDrone(droneID=targetID)
            # if deletionAttempt.startswith("Successfully deleted entity"):
            targetThreat.currentStatus="Dead"
            
            ic(f"Drone {targetID} Destroyed With {weaponModel.weaponName}")


def simulate_BOWSER_simulation(spawnRange:set, algorithmChoice:str, numberOfDrones:int, threatCoordinates=None) -> tuple:
    global SYSTEM_RUNNING
    
    threatFileLocation = "dataFiles/simulationDroneLocations.csv"
    weaponFileLocation = "dataFiles/weapon_data.csv"
    dqnModelPath = "dataFiles/trained_model.zip"
    
    
    droneDirectory, locationDirectory, subprocesses = {}, {}, {}
    

            
    print(" ----------------------------- Welcome to the BOWSER Prototype ----------------------------- ")
    print(" ------------------------------------- No Gazebo / ROS -------------------------------------- ")
   
    print(f"You have selected {numberOfDrones} Drones")
    ic(algorithmChoice)
    
    ic('doing weapon things')
    weaponList = initializeWeaponsFromFile(weapon_file=weaponFileLocation)
    
    print(f"Simulating {numberOfDrones} Drones")

    for droneID in range(numberOfDrones):
        if spawnRange[0] > spawnRange[1]:
            raise ValueError("Minimum radius cannot be greater than maximum radius")
        
        if threatCoordinates and droneID+1 in threatCoordinates:
            # Use user-defined coordinates
            startingX = threatCoordinates[droneID+1]["x"]
            startingY = threatCoordinates[droneID+1]["y"]
            # startingZ = threatCoordinates[droneID+1]["z"]
            startingZ = 0
        else:
            # Generate random angle within 0 to 2*pi
            angle = uniform(0, 2 * pi)
            # Generate random radius within the range [min_radius, max_radius]
            radius = uniform(spawnRange[0], spawnRange[1])
            # Calculate x and y coordinates using polar coordinates to Cartesian coordinates conversion
            startingX = radius * cos(angle)
            startingY = radius * sin(angle)
            startingZ = 0

        # Starting Location  = [x,y,z]
    
        locationDirectory[droneID] = {"x":startingX,"y":startingY,"z":startingZ, "minRange": spawnRange[0]}    
        tempDrone = Drone(droneID=droneID, 
                      currentStatus= "Alive",
                      startingLocation=[startingX,startingY,startingZ],
                      currentLocation=[startingX,startingY,startingZ])
        droneDirectory[droneID] = tempDrone    
    
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
        algorithm_leaker_percentage *= 100
    
    elif algorithmChoice == "ACO":
        from algorithms.ACO import runACO
        response,leaker_percentage = runACO(threatFileLocation=threatFileLocation)
        leaker_percentage = (1.00 - leaker_percentage) * 100
    
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
    ic(response)
    ic(weaponList)
    
    # for the time being, The algorithms do not change the order in which the threats are entered, so their index should be the same as the ID
    for droneID, target in enumerate(response):
        # if assigned weapon is short range, add to the short range targets 
        if target[1][0].lower() == weaponList[0].weaponName:
            targetDelegations[0].append(int(droneID)) 
        # if assigned weapon is medium range, add to the medium range targets 
        elif target[1][0].lower() == weaponList[1].weaponName:
            targetDelegations[1].append(int(droneID)) 
        # if assigned weapon is long range, add to the long range targets 
        elif target[1][0].lower() == weaponList[2].weaponName:
            targetDelegations[2].append(int(droneID)) 
        # if none else, it should be a directed energy target, thus add to the D.E targets 
        else:
            targetDelegations[3].append(int(droneID))  
      
    ic(targetDelegations)
    weaponThreads = []  
    for weaponIndex, weaponSystem in enumerate(weaponList):
        weaponThread = Thread(target=startWeaponSystem, args=(weaponSystem, targetDelegations[weaponIndex],(0,0,0), droneDirectory))
        weaponThread.start()
        weaponThreads.append(weaponThread)
        
    for thread in weaponThreads:
        thread.join()
        

    ic("No more Subprocesses, all robots should be dead")

    simulation_leaker_percent = (LEAKER_COUNT / numberOfDrones) * 100 
    ic(simulation_leaker_percent, algorithm_leaker_percentage)
  
    return simulation_leaker_percent, algorithm_leaker_percentage

if __name__ == "__main__":
        simulate_BOWSER_simulation(spawnRange=(10,20), algorithmChoice="DQN", numberOfDrones=3)
