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
        # # ic(list(readerObj))
        for weaponID, threatEntry in enumerate(list(readerObj)):
            # # ic(threatEntry)
            weaponPlacement = Weapon(weaponID=weaponID,
                                     weaponName=threatEntry[0],
                                     location=[threatEntry[1], threatEntry[2], threatEntry[3]],
                                     ammunitionQuantity=int(threatEntry[5]),
                                     firingRate=float(threatEntry[6])
                                    )
            weapons.append(weaponPlacement)
        
        return weapons


def startWeaponSystem(weaponModel:Weapon, targetList:list, placement:set, droneDirectory:dict) -> None:
    for idx, targetID in enumerate(targetList):
        if targetID != 'x':
            targetThreat = droneDirectory[targetID]
            if targetThreat.currentStatus == "Alive":
                if weaponModel.ammunitionQuantity <= 0:
                    ic(f"{weaponModel.weaponName} has run out of ammunition")
                    ic(f"Remaining Targets for {weaponModel.weaponName}: {targetList}")
                    return
                    
                ic(f"{weaponModel.weaponName} has fired once at Drone {targetID}")
                weaponModel.ammunitionQuantity -= 1
                targetThreat.currentStatus="Dead"
                
                ic(f"Drone {targetID} Destroyed With {weaponModel.weaponName}")
                targetList[idx] = 'x'
                sleep(weaponModel.firingRate)    
    
    


def robotController(robot:Threat) -> None:
    global LEAKER_COUNT
    threatId = robot.threatID
    startingX = robot.startingLocation[0]
    startingY = robot.startingLocation[1]
    startingZ = robot.startingLocation[2] # Currently 0 as simulation uses ground robots instead of drones
    speed = robot.speed
    leakerRange = robot.leakerRange
    
    
    acceleration = 5.0 # m/s
    accelerationTime = speed / acceleration
    accelerationDistance = 0.5 * acceleration * accelerationTime**2
    
    
    
    distanceUntilLeaker = (((startingX**2 + startingY**2 + startingZ**2)**0.5) - accelerationDistance) - leakerRange
    
    timeToReachOrigin = distanceUntilLeaker / speed
    
    
    
    startTime = time()
    while robot.currentStatus == "Alive" and (accelerationTime + timeToReachOrigin) > (time() - startTime):
        sleep(0.02)
    if robot.currentStatus == "Alive":
        # ic(f"Threat {threatId} has reached the origin and is a leaker")
        robot.currentStatus = "Leaker"
        LEAKER_COUNT += 1
        return
    

    # ic(f"Threat {threatId} Thread Completed")
    return    


def simulate_BOWSER_simulation (algorithm:str, threatDirectory:dict) -> tuple:
    global SYSTEM_RUNNING, LEAKER_COUNT
    
    weaponFileLocation = "dataFiles/weapon_data.csv"
    dqnModelPath = "dataFiles/trained_model.zip"
    threatFileLocation = "dataFiles/simulationThreatLocations.csv"

    numberOfThreats = len(threatDirectory)
    

    print(" ----------------------------- Welcome to the BOWSER Prototype ----------------------------- ")
    print(" ---------------------------------- Gazebo-less Simulation --------------------------------- ")
   
    print(f"You have selected {numberOfThreats} Threats")
    print(f"Using {algorithm} Algorithm")
     
    weaponList = initializeWeaponsFromFile(weapon_file=weaponFileLocation)
    SYSTEM_RUNNING = True
    
    print(f"Spawn in {numberOfThreats} Drones")
    threads = []
    for x in range(numberOfThreats):
        # Create a thread for each drone
        thread = Thread(target=robotController, args=(threatDirectory[x],))
        thread.start()
        threads.append(thread)
        
        
        
        
        
    
    """Call Algorithm """    
    start = time()
    if algorithm == "DQN":
        from algorithms.dqn_agent import runDQN
        runDQN(savePath=dqnModelPath, train=True, num_threats=numberOfThreats)
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
    
    elif algorithm == "Ant Colony":
        from algorithms.ACO import runACO
        response, algorithm_leaker_percentage = runACO(threatFileLocation=threatFileLocation)
        algorithm_leaker_percentage = (1.00 - algorithm_leaker_percentage) * 100
    
    elif algorithm == "Particle Swarm":
        from algorithms.Particle_Swarm_Optimization import runPSO
        response, algorithm_leaker_percentage = runPSO(threatFileLocation=threatFileLocation)
        algorithm_leaker_percentage = (1.00 - algorithm_leaker_percentage) * 100
    
    else:
        raise "Invalid Algorithm Choice"
    end = time()
    
    """Handle Drones in Simulation According to Algorithm Results"""
    # ic(threatDirectory)
    
    """
    Assigning Targets to defense weaponry, right now assuming there are 1 of each weapon system, all located at (0,0)
    """
    # [[short range targets],[medium Range Targets], [Long Range Targets], [Directed Energy Targets]] 
    targetDelegations = [[],[],[],[]]
    # ic(weaponList)
    ic(response)
    
    for weaponEntry in range(len(response[0][1])): 
        # for the time being, The algorithms do not change the order in which the threats are entered, so their index should be the same as the ID
        for droneID, target in enumerate(response):
            try:
                # if assigned weapon is short range, add to the short range targets 
                if target[1][weaponEntry].lower() == weaponList[0].weaponName:
                    targetDelegations[0].append(int(droneID)) 
                # if assigned weapon is medium range, add to the medium range targets   
                elif target[1][weaponEntry].lower() == weaponList[1].weaponName:
                    targetDelegations[1].append(int(droneID)) 
                # if assigned weapon is long range, add to the long range targets 
                elif target[1][weaponEntry].lower() == weaponList[2].weaponName:
                    targetDelegations[2].append(int(droneID)) 
                # if none else, it should be a directed energy target, thus add to the D.E targets 
                else:
                    targetDelegations[3].append(int(droneID))  
            except Exception as e:
                
                ic(e, target, weaponEntry, droneID,  (len(response[1])))
        # ic(targetDelegations)
        weaponThreads = []  
        for weaponIndex, weaponSystem in enumerate(weaponList):
            weaponThread = Thread(target=startWeaponSystem, args=(weaponSystem, targetDelegations[weaponIndex],(0,0,0), threatDirectory))
            weaponThread.start()
            weaponThreads.append(weaponThread)
        for thread in weaponThreads:
            thread.join()
    
    
    for threat in threatDirectory:
        if threatDirectory[threat].currentStatus == "Alive":
            # ic(f"Threat {threat} has survived and leaked")
            threatDirectory[threat].currentStatus = "Leaker"
            LEAKER_COUNT += 1
    

    
    
    
   


    
    

    simulation_leaker_percent = (LEAKER_COUNT / numberOfThreats) * 100 
    # ic(simulation_leaker_percent, algorithm_leaker_percentage, end-start)
    return algorithm_leaker_percentage, simulation_leaker_percent






    
if __name__ == "__main__":
    # # ic("please run full program. no test location dict created yet")
    dummyDroneDirectory = {
                       0: Threat(threatID=0, currentStatus='Alive', leakerRange=1.0, speed=1000.0, startingLocation=[26.00580291816817, -44.105597561488764, 0], destroyedLocation=[], currentLocation=[]),
                      1: Threat(threatID=1, currentStatus='Alive', leakerRange=10.0, speed=200.0, startingLocation=[-51.112840604320986, 1.6490815897669158, 0], destroyedLocation=[], currentLocation=[]),
                      2: Threat(threatID=2, currentStatus='Alive', leakerRange=1.0, speed=225.0, startingLocation=[-33.3218579619414, -2.6078985893150333, 0], destroyedLocation=[], currentLocation=[]),
                      3: Threat(threatID=3, currentStatus='Alive', leakerRange=1.0, speed=1000.0, startingLocation=[-1.2628347012036054, -22.476651880389106, 0], destroyedLocation=[], currentLocation=[]),
                      4: Threat(threatID=4, currentStatus='Alive', leakerRange=1.0, speed=1000.0, startingLocation=[-50.152301163669485, -38.35252885137785, 0], destroyedLocation=[], currentLocation=[]),
                      5: Threat(threatID=5, currentStatus='Alive', leakerRange=10.0, speed=200.0, startingLocation=[-54.084484436000025, -37.32411535940071, 0], destroyedLocation=[], currentLocation=[]),
                      6: Threat(threatID=6, currentStatus='Alive', leakerRange=10.0, speed=200.0, startingLocation=[-22.395577976263024, -13.930194897777735, 0], destroyedLocation=[], currentLocation=[]),
                      7: Threat(threatID=7, currentStatus='Alive', leakerRange=1.0, speed=225.0, startingLocation=[-50.37019795443521, -20.871748208822613, 0], destroyedLocation=[], currentLocation=[]),
                      8: Threat(threatID=8, currentStatus='Alive', leakerRange=1.0, speed=1000.0, startingLocation=[-19.89508920576726, -2.3314013481701883, 0], destroyedLocation=[], currentLocation=[]),
                      9: Threat(threatID=9, currentStatus='Alive', leakerRange=1.0, speed=225.0, startingLocation=[-39.800291292351275, -52.95114983203683, 0], destroyedLocation=[], currentLocation=[]),
                      10: Threat(threatID=10, currentStatus='Alive', leakerRange=10.0, speed=200.0, startingLocation=[-36.26243169344489, -52.935250397807586, 0], destroyedLocation=[], currentLocation=[]),
                      11: Threat(threatID=11, currentStatus='Alive', leakerRange=1.0, speed=1000.0, startingLocation=[10.999749047833157, -25.47669494926166, 0], destroyedLocation=[], currentLocation=[]),
                      12: Threat(threatID=12, currentStatus='Alive', leakerRange=1.0, speed=1000.0, startingLocation=[-62.983260736967964, -10.686763174031013, 0], destroyedLocation=[], currentLocation=[])
                        }
    
    for _ in range(10):
        ic(simulate_BOWSER_simulation(algorithm="DQN", threatDirectory=dummyDroneDirectory))
