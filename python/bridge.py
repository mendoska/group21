from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
##import torch
##torch.zeros(1).cuda()

threatFileLocation="dataFiles/threat_location.csv"
weaponFileLocation="dataFiles/weapon_data.csv"
dqnModelPath = "dataFiles/trained_model.zip"

def selectAlgorithm():
    algorithmChoice = input("""
    Please Input 1 - 4 to Select Algorithm:
    ---------------------------------------
    1: Deep Q Network
    2: Genetic Algorithm
    3: Munkres (Hungarian)
    4: Simulated Annealing \n""")


    if algorithmChoice == "1":
        runDQN(savePath=dqnModelPath, train=True)
    elif algorithmChoice == "2":
        runDQN(loadPath=dqnModelPath, train=False)
    elif algorithmChoice == "3":
        runGA(threatFileLocation=threatFileLocation)
    elif algorithmChoice == "4":
        runMunkres(threatFileLocation=threatFileLocation, weaponFileLocation=weaponFileLocation)
    elif algorithmChoice == "5":
        runSimulatedAnnealing()
    
# selectAlgorithm()   