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


def simulate_BOWSER_simulation(numberOfDrones:int, algoResponse:list, droneDirectory:dict, locationDirectory:dict) -> tuple:
    global SYSTEM_RUNNING
    
    weaponFileLocation = "dataFiles/weapon_data.csv"
    
    
    subprocesses = {}
    

            
    print(" ----------------------------- Welcome to the BOWSER Prototype ----------------------------- ")
    print(" ------------------------------------- No Gazebo / ROS -------------------------------------- ")
   
    print(f"You have selected {numberOfDrones} Drones")
    
    ic('doing weapon things')
    weaponList = initializeWeaponsFromFile(weapon_file=weaponFileLocation)
    
    print(f"Simulating {numberOfDrones} Drones")

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
    for droneID, target in enumerate(algoResponse):
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
  
    return simulation_leaker_percent

#if __name__ == "__main__":
    # simulate_BOWSER_simulation(spawnRange=(10,20), algorithmChoice="DQN", numberOfDrones=3)
