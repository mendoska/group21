from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing


threatFileLocation="dataFiles/threat_location.csv"
weaponFileLocation="dataFiles/weapon_data.csv"


def selectAlgorithm():
    algorithmChoice = input("""
    Please Input 1 - 4 to Select Algorithm:
    ---------------------------------------
    1: Deep Q Network
    2: Genetic Algorithm
    3: Munkres (Hungarian)
    4: Simulated Annealing \n""")


# # If running python 3.10, use better match case vs elif
# match algorithmChoice:
#     case "1":
#         runDQN()
#     case "2":
#         runGA(threatFileLocation=threatFileLocation)
#     case "3":
#         runMunkres(threatFileLocation=threatFileLocation,weaponFileLocation=weaponFileLocation)
#     case "4":
#         runSimulatedAnnealing

    if algorithmChoice == "1":
        runDQN()
    elif algorithmChoice == "2":
        runGA(threatFileLocation=threatFileLocation)
    elif algorithmChoice == "3":
        runMunkres(threatFileLocation=threatFileLocation,weaponFileLocation=weaponFileLocation)
    elif algorithmChoice == "4":
        runSimulatedAnnealing
    
selectAlgorithm()