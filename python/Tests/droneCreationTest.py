from Models.Drone import Drone
from icecream import ic

droneDirectory = {}

for x in range(5):
    tempDrone = Drone(droneID=x)
    droneDirectory[x]=(tempDrone)
    
ic(droneDirectory)


