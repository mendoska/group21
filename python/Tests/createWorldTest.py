from icecream import ic
from CLIFunctions.cliFunctions import startEmptyGazeboWorldSimulation
import asyncio
import subprocess


response =  asyncio.run(startEmptyGazeboWorldSimulation(totalNumberOfDrones=5))

ic(response)


