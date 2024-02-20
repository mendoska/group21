from icecream import ic
from CLIFunctions.cliFunctions import startEmptyGazeboWorldSimulation
import asyncio
import subprocess


worldCreationProcess = startEmptyGazeboWorldSimulation(totalNumberOfDrones=2)
line = worldCreationProcess.stdout.readline()

while(not line.startswith("[INFO] [ground_truth_publisher-2]: process started with pid")):
    ic(line)
    line = worldCreationProcess.stdout.readline()
    
ic("World Created")
    



