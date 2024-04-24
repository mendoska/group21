from Models.Threat import Threat
from Models.Weapon import Weapon
from CLIFunctions.cliFunctions import addThreatToSimulation, issueStartCommand, startEmptyGazeboWorldSimulation
from CLIFunctions.cliFunctions import destroyThreat, stopWorldSimulation
from threading import Thread, Lock, Condition
from icecream import ic
from time import time, sleep
from subprocess import Popen
from csv import  reader
from yaml import safe_load, dump


NO_SIM_MODE = False


START_FLAG = 0
SYSTEM_RUNNING = False
START_COUNTER = 0
LEAKER_COUNT = 0

# global thread lock
file_lock = Lock()

# Define a condition variable
file_condition = Condition(file_lock)


def initializeWeaponsFromFile(weapon_file:str)-> list:
    with open(weapon_file, 'r') as file:
        weapons = []
        readerObj = reader(file)
        for weaponID, threatEntry in enumerate(list(readerObj)):
            weaponPlacement = Weapon(weaponID=weaponID,
                                     weaponName=threatEntry[0],
                                     location=[threatEntry[1], threatEntry[2], threatEntry[3]],
                                     ammunitionQuantity=int(threatEntry[5]),
                                     firingRate=float(threatEntry[6])
                                    )
            weapons.append(weaponPlacement)
        
        return weapons


def startWeaponSystem(weaponModel:Weapon, targetList:list, placement:set, threatDirectory:dict) -> None:
    global LEAKER_COUNT
    for idx, targetID in enumerate(targetList):
        targetThreat = threatDirectory[targetID]
        # ic(weaponModel.weaponName,threatDirectory,targetThreat,targetList)
        if targetThreat.currentStatus == "Alive":
            if weaponModel.ammunitionQuantity <= 0:
                ic(f"{weaponModel.weaponName} has run out of ammunition")
                ic(f"Remaining Targets for {weaponModel.weaponName}: {targetList}")
                return
                
            ic(f"{weaponModel.weaponName} has fired once at Drone {targetID}")
            weaponModel.ammunitionQuantity -= 1
            deletionAttempt = destroyThreat(threatID=targetID)
            if deletionAttempt.startswith("Successfully deleted entity"):
                targetThreat.currentStatus="Dead"
            
            ic(f"Drone {targetID} Destroyed With {weaponModel.weaponName}")
            targetList[idx] = 'x'
            sleep(weaponModel.firingRate)



def alterD2OSpawnParameters(s:float = 1.0, lr:float = 1.0, threatID=int ) -> None:
    
    filePath = '../simulation/swarm_ws/ROS2swarm_B/install/ros2swarm/share/ros2swarm/config/waffle_pi/movement_pattern/basic/drive2OriginPattern.yaml'
    with open(file=filePath,mode='r') as file:
        data = safe_load(file)
    # ic(data["/**"]["ros__parameters"])
    data["/**"]["ros__parameters"]["speed"] = s
    data["/**"]["ros__parameters"]["leaker_range"] = lr
    ic(threatID,s, lr,data["/**"]["ros__parameters"])
    # Write back to YAML file
    with open(file=filePath, mode='w') as file:
        dump(data, file)

"""

Constantly reads the stdout of the provided subprocess, waiting for the target line to appear.
options: 
        0 - Does Not Print Lines (Defualt)    
        1 - Print Lines While Waiting For Target Line (Debugging Purposes)
    
"""
def waitForLineInSubprocess(subprocess:Popen, targetStr:str, option:int = 0, threatModel:Threat = None):
    line = subprocess.stdout.readline()
    while(not targetStr in line):
        if threatModel != None:
            if threatModel.currentStatus != "Alive":
                return
        if option == 1:
            if line != '':
                ic(line) # prints current line to verify what process is doing
        line = subprocess.stdout.readline()
    print(threatModel.threatID if threatModel else "",line)
    return


def delayedStartCommand(numberOfThreats:int) -> None:
    global START_COUNTER, START_FLAG
    
    # while waiting for all the subprocesses to be created, the delayedStartCommand does nothing
    while START_COUNTER != numberOfThreats:
        continue
    # # Once all of the drones have been spawned in, the function waits 10 seconds (arbitrary), and issues the start command, causing all drones to travel to the center
    sleep(10)
    issueStartCommand()
    # the start flag is sent, allowing all subprocesses to terminate and kill themselves
    START_FLAG = 1
    return

"""
starts the subprocess which contains the robot/drone's "brain"
waits until robot/drone has reached (0,0) +/- the leaker range and 
then increments the leaker count, updates the drone's class model, and removes the drone from the sim
"""
def robotController(robot:Threat, subprocesses:dict) -> None:
    global START_COUNTER, LEAKER_COUNT

    threatID = robot.threatID
    startingX = robot.startingLocation[0]
    startingY = robot.startingLocation[1]
    startingZ = robot.startingLocation[2] # Currently 0 as simulation uses ground robots instead of drones
    speed = robot.speed
    leakerRange = robot.leakerRange
    
    
    with file_lock:
        print(f"Spawning Drone at ({startingX},{startingY},{startingZ}), Speed: {speed}, Leaker Range: {leakerRange}")
        alterD2OSpawnParameters(s=speed, lr=leakerRange, threatID=threatID)

        # calls the function which creates and returns a subprocess object which executes the "add_robots_to_simulation" command
        threatSubprocess = addThreatToSimulation(threatID=threatID, spawnCoordinateX=startingX, spawnCoordinateY=startingY)
        waitForLineInSubprocess(subprocess=threatSubprocess, targetStr=f"Leaker Range is: {leakerRange}", option=0)
        file_condition.notify_all()
    # once the subprocess is successfully created, the global counter is incremented and the drone's information is stored
    waitForLineInSubprocess(subprocess=threatSubprocess, targetStr="[INFO] [add_bot_node-1]: process has finished cleanly [pid", option=0)
    START_COUNTER += 1
        
    subprocesses[f"robot_{threatID}"] = threatSubprocess
    
    # wait for signal that drone has reached the origin, then begin the process of deleting robot (Checks twice as a bandaid fix because false flags have appeared)
    waitForLineInSubprocess(subprocess=threatSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=0, threatModel=robot)
    waitForLineInSubprocess(subprocess=threatSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=0, threatModel=robot)
    if robot.currentStatus == "Alive":
        robot.currentStatus="Leaker"
        LEAKER_COUNT += 1
        destroyThreat(threatID=threatID)
    subprocesses.pop(f"robot_{threatID}")
    
    # Cleaning up subprocess, (terminate and kill it) 
    threatSubprocess.terminate()
    sleep(3)
    threatSubprocess.kill()
    ic(f"Threat {threatID} Process Killed")
    ic()
    ic("Done")
    return 
    
    
    
    

def run_BOWSER_simulation(algorithm:str, threatDirectory:dict) -> tuple:
    global SYSTEM_RUNNING, LEAKER_COUNT
    
    weaponFileLocation = "dataFiles/weapon_data.csv"
    dqnModelPath = "dataFiles/trained_model.zip"
    threatFileLocation = "dataFiles/simulationThreatLocations.csv"

    numberOfThreats = len(threatDirectory)
    subprocesses = {}
    

    print(" ----------------------------- Welcome to the BOWSER Prototype ----------------------------- ")
   
    print(f"You have selected {numberOfThreats} Threats")
    
    ic("Start Empty Gazebo World")
    worldCreationProcess = startEmptyGazeboWorldSimulation(totalNumberOfDrones=numberOfThreats)
    waitForLineInSubprocess(subprocess=worldCreationProcess, targetStr="[INFO] [ground_truth_publisher-2]: process started with pid", option=1)
    ic("World Created")    
    weaponList = initializeWeaponsFromFile(weapon_file=weaponFileLocation)
    SYSTEM_RUNNING = True
    
    print(f"Spawn in {numberOfThreats} Drones")
    threads = []
    for x in range(numberOfThreats):
        # Create a thread for each drone
        thread = Thread(target=robotController, args=(threatDirectory[x], subprocesses))
        thread.start()
        threads.append(thread)
        
        
    # Once each drone thread has been created, A final "timer" thread is started which monitors the number of threads that have been executed
    # Once all of the threads have begun executing, this thread will wait a moment and then send the start command, which will then prompt the drone
    # threads to terminal and kill the subprocesses, allowing the algorithms and such to begin
    startDroneThread = Thread(target=delayedStartCommand,args=(numberOfThreats,))
    startDroneThread.start()
    threads.append(startDroneThread)

    # Wait for threads to finish before continuing 
    while START_FLAG != 1:
        continue
    
    
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
    ic(threatDirectory)
    ic(subprocesses)
    
    """
    Assigning Targets to defense weaponry, right now assuming there are 1 of each weapon system, all located at (0,0)
    """
    # [[short range targets],[medium Range Targets], [Long Range Targets], [Directed Energy Targets]] 
    targetDelegations = [[],[],[],[]]
    ic(weaponList)
    
    # for the time being, The algorithms do not change the order in which the threats are entered, so their index should be the same as the ID
    for weaponEntry in range(len(response[0][1] )): 
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
    ic(simulation_leaker_percent, algorithm_leaker_percentage, end-start)
    return algorithm_leaker_percentage, simulation_leaker_percent






if __name__ == "__main__":
    # ic("please run full program. no test location dict created yet")
    
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
    
    
    run_BOWSER_simulation(algorithm="Particle Swarm", threatDirectory=dummyDroneDirectory)
    # print("s")


