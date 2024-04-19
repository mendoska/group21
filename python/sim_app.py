from Models.Threat import Threat
from Models.Weapon import Weapon
from icecream import ic
from threading import Thread
from time import time, sleep
from csv import reader

LEAKER_COUNT = 0


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
            targetThreat.currentStatus="Dead"
            
            ic(f"Drone {targetID} Destroyed With {weaponModel.weaponName}")


def simulate_BOWSER_simulation(algorithm:str, droneDirectory:dict, locationDirectory:dict) -> tuple:
    global SYSTEM_RUNNING
    
    weaponFileLocation = "dataFiles/weapon_data.csv"
    dqnModelPath = "dataFiles/trained_model.zip"
    threatFileLocation = "dataFiles/simulationDroneLocations.csv"
    
    numberOfDrones = len(droneDirectory)
    subprocesses = {}
    

            
    print(" ----------------------------- Welcome to the BOWSER Prototype ----------------------------- ")
    print(" ------------------------------------- No Gazebo / ROS -------------------------------------- ")
   
    print(f"You have selected {numberOfDrones} Drones")
    
    ic('doing weapon things')
    weaponList = initializeWeaponsFromFile(weapon_file=weaponFileLocation)
    
    print(f"Simulating {numberOfDrones} Drones")


    """Call Algorithm """    
    start = time()
    if algorithm == "DQN":
        from algorithms.dqn_agent import runDQN
        runDQN(savePath=dqnModelPath, train=True, num_threats=numberOfDrones)
        response, algorithm_leaker_percentage = runDQN(loadPath=dqnModelPath, train=False, threatFilePath=threatFileLocation)
    
    elif algorithm == "Genetic Algorithm":
        from algorithms.geneticAlgorithmTest import runGA
        response, algorithm_leaker_percentage = runGA(threatFileLocation=threatFileLocation)
        algorithm_leaker_percentage *= 100

    elif algorithm == "Munkres":
        from algorithms.munkres_algorithm import runMunkres
        response, algorithm_leaker_percentage = runMunkres(threatFileLocation=threatFileLocation, weaponFileLocation=weaponFileLocation)

    elif algorithm == "Simulated Annealing":
        from algorithms.simulated_annealing import runSimulatedAnnealing
        response, algorithm_leaker_percentage = runSimulatedAnnealing()
        algorithm_leaker_percentage *= 100
    
    elif algorithm == "ACO":
        from algorithms.ACO import runACO
        response, algorithm_leaker_percentage = runACO(threatFileLocation=threatFileLocation)
        algorithm_leaker_percentage = (1.00 - algorithm_leaker_percentage) * 100
    
    else:
        raise "Invalid Algorithm Choice"
    end = time()

    # print(f"Waiting {(end-start)} seconds to offset Sim Time to Real Time difference")
    # sleep(end-start)


    """Handle Drones in Simulation According to Algorithm Results"""
    ic(droneDirectory)
    ic(subprocesses)
    
    """
    Assigning Targets to defense weaponry, right now assuming there are 1 of each weapon system, all located at (0,0)
    """
    # [[short range targets],[medium Range Targets], [Long Range Targets], [Directed Energy Targets]] 
    targetDelegations = [[],[],[],[]]
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
    ic(simulation_leaker_percent)
  
    return algorithm_leaker_percentage, simulation_leaker_percent
    
if __name__ == "__main__":
    # ic("please run full program. no test location dict created yet")
    dummyDroneDirectory = {0: Drone(droneID=0, currentStatus='Alive', startingLocation=[-0.1633032204811115, 13.401510241791964, 0], destroyedLocation=[], currentLocation=[-0.1633032204811115, 13.401510241791964, 0]),
                      1: Drone(droneID=1, currentStatus='Alive', startingLocation=[-0.06627079078009374, 17.63481676928732, 0], destroyedLocation=[], currentLocation=[-0.06627079078009374, 17.63481676928732, 0]),
                      2: Drone(droneID=2, currentStatus='Alive', startingLocation=[-0.017752763251043694, 10.967149399952069, 0], destroyedLocation=[], currentLocation=[-0.017752763251043694, 10.967149399952069, 0]),
                      3: Drone(droneID=3, currentStatus='Alive', startingLocation=[-0.19041370954388556, 18.070279528163976, 0], destroyedLocation=[], currentLocation=[-0.19041370954388556, 18.070279528163976,0]),
                      4: Drone(droneID=4, currentStatus='Alive', startingLocation=[-0.18466013793661437, 19.730293521380943, 0], destroyedLocation=[], currentLocation=[-0.18466013793661437, 19.730293521380943,0])}
    
    dummyLocationDirectory = {0: {'Speed': 225.0,
                             'minRange': 100.0,
                             'x': -0.1633032204811115,
                             'y': 13.401510241791964,
                             'z': 0},
                         1: {'Speed': 225.0,
                             'minRange': 100.0,
                             'x': -0.06627079078009374,
                             'y': 17.63481676928732,
                             'z': 0},
                         2: {'Speed': 200.0,
                             'minRange': 90000.0,
                             'x': -0.017752763251043694,
                             'y': 10.967149399952069,
                             'z': 0},
                         3: {'Speed': 2250.0,
                             'minRange': 500.0,
                             'x': -0.19041370954388556,
                             'y': 18.070279528163976,
                             'z': 0},
                         4: {'Speed': 270.0,
                             'minRange': 50000.0,
                             'x': -0.18466013793661437,
                             'y': 19.730293521380943,
                             'z': 0}}
    
    simulate_BOWSER_simulation(algorithm="Simulated Annealing", droneDirectory=dummyDroneDirectory, locationDirectory=dummyLocationDirectory)
