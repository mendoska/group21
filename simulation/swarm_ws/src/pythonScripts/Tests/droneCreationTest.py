from Models.Drone import Drone
from CLIFunctions.cliFunctions import addDroneToSimulation
import asyncio
from icecream import ic

droneDirectory = {}
summonNumberOfDrones = 1
i = 1
for x in range(summonNumberOfDrones):
    addDroneToSimulation(droneID=i)
    tempDrone = Drone(droneID=i)
    droneDirectory[x]=(tempDrone)
    
ic(droneDirectory)



