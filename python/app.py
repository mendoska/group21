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
                                     firingRate=threatEntry[6]
                                    )
            weapons.append(weaponPlacement)
        
        return weapons


def startWeaponSystem(weaponModel:Weapon, targetList:list, placement:set, threatDirectory:dict) -> None:
    global LEAKER_COUNT
    for targetID in targetList:
        targetThreat = threatDirectory[targetID]
        ic(threatDirectory,targetThreat)
        if targetThreat.currentStatus == "Alive":
            if weaponModel.ammunitionQuantity <= 0:
                ic(f"{weaponModel.weaponName} has run out of ammunition, Drone {targetID} as become a leaker")
                targetThreat.currentStatus = "Leaker"
                LEAKER_COUNT += 1
                
            ic(f"{weaponModel.weaponName} has fired once at Drone {targetID}")
            weaponModel.ammunitionQuantity -= 1
            deletionAttempt = destroyThreat(threatID=targetID)
            if deletionAttempt.startswith("Successfully deleted entity"):
                targetThreat.currentStatus="Dead"
            
            ic(f"Drone {targetID} Destroyed With {weaponModel.weaponName}")



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
    print(line)
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
def robotController(threatID:int, threatDirectory:dict, subprocesses:dict) -> None:
    global START_COUNTER, LEAKER_COUNT

    robot = threatDirectory[threatID]
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
        waitForLineInSubprocess(subprocess=threatSubprocess, targetStr=f"Leaker Range is: {leakerRange}", option=1)
        # once the subprocess is successfully created, the global counter is incremented and the drone's information is stored
        waitForLineInSubprocess(subprocess=threatSubprocess, targetStr="[INFO] [add_bot_node-1]: process has finished cleanly [pid", option=0)
        file_condition.notify_all()
    START_COUNTER += 1
        
    subprocesses[f"robot_{threatID}"] = threatSubprocess
    
    # wait for signal that drone has reached the origin, then begin the process of deleting robot (Checks twice as a bandaid fix because false flags have appeared)
    waitForLineInSubprocess(subprocess=threatSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=0, threatModel=robot)
    waitForLineInSubprocess(subprocess=threatSubprocess, targetStr="[drive2OriginPattern-4] ic| 'Robot Has Leaked, Commence Deletion'", option=1, threatModel=robot)
    if threatDirectory[threatID].currentStatus == "Alive":
        threatDirectory[threatID].currentStatus="Leaker"
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
    global SYSTEM_RUNNING
    
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
        thread = Thread(target=robotController, args=(x,threatDirectory, subprocesses))
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
    
    elif algorithm == "ACO":
        from algorithms.ACO import runACO
        response, algorithm_leaker_percentage = runACO(threatFileLocation=threatFileLocation)
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
        weaponThread = Thread(target=startWeaponSystem, args=(weaponSystem, [weaponIndex],(0,0,0), threatDirectory))
        weaponThread.start()
        weaponThreads.append(weaponThread)
        
    for thread in weaponThreads:
        thread.join()
            
    
     
    # ic(threatDirectory)
    # ic(subprocesses)
    lc = LEAKER_COUNT
    while LEAKER_COUNT + len(subprocesses) != numberOfThreats:
        if lc != LEAKER_COUNT:
            ic(LEAKER_COUNT)
            lc = LEAKER_COUNT
        continue
    

    
    try:
        for subprocessID in subprocesses:
            # ic(subprocesses)
            ic(subprocesses[subprocessID])
            subprocesses[subprocessID].terminate()
            sleep(1)
            subprocesses[subprocessID].kill()
    except Exception as e: 
        ic(e)
        pass
    
    stopWorldSimulation()


    
    

    simulation_leaker_percent = (LEAKER_COUNT / numberOfThreats) * 100 
    ic(simulation_leaker_percent, algorithm_leaker_percentage, end-start)
    return algorithm_leaker_percentage, simulation_leaker_percent






if __name__ == "__main__":
    # ic("please run full program. no test location dict created yet")
    
    dummyDroneDirectory = {
                      
                      0: Threat(threatID=0, currentStatus='Alive', leakerRange=1.0, speed=50.0, startingLocation=[-61.861562065612226, 7.25152907262536, 0], destroyedLocation=[], currentLocation=[]),
                      
                          }
    
    
    run_BOWSER_simulation(algorithm="Simulated Annealing", threatDirectory=dummyDroneDirectory)
    # print("s")


