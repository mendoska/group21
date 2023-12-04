from Models.Drone import Drone
from CLIFunctions.cliFunctions import addDroneToSimulation
import asyncio
from icecream import ic

droneDirectory = {}
summonNumberOfDrones = 4
for x in range(summonNumberOfDrones):
    addDroneToSimulation(droneID=x)
    tempDrone = Drone(droneID=x)
    droneDirectory[x]=(tempDrone)
    
ic(droneDirectory)



