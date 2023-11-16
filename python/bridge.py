from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from time import time


threatFileLocation="dataFiles/threat_location.csv"
weaponFileLocation="dataFiles/weapon_data.csv"


def selectAlgorithm(algorithmChoice=None):
    if algorithmChoice == None:
        algorithmChoice = input("""
        Please Input 1 - 4 to Select Algorithm:
        ---------------------------------------
        1: Deep Q Network
        2: Genetic Algorithm
        3: Munkres (Hungarian)
        4: Simulated Annealing \n""")


    if algorithmChoice == "1":
        runDQN()
    elif algorithmChoice == "2":
        res = runGA()
        printOutputList(outputList=res)
    elif algorithmChoice == "3":
        res = runMunkres(threatFileLocation=threatFileLocation,weaponFileLocation=weaponFileLocation)
        printOutputList(outputList=res)    
    elif algorithmChoice == "4":
        res = runSimulatedAnnealing()
        printOutputList(outputList=res)    
    else:
        print("Invalid Input")



def printOutputList(outputList: list):
    for entry in outputList:
        print(entry)
    

def testTimeForSingleAlgorithm(selection:str) -> float:
    start_time = time()
    selectAlgorithm(selection)
    finish_time = time()
    return finish_time - start_time

def testTimeAllArrays():
    r = []
    for i in range(4):
        r.append(f'{i}: {testTimeForSingleAlgorithm(f"{i+1}")} Seconds')
    printOutputList(outputList=r)
    




selectAlgorithm()

