from icecream import ic
from fakeSimulationFunctions import simulateAddingDrone, writeDictToCSV
from Models.Drone import Drone
from bridge import selectAlgorithm
from time import time


def getUserInput(prompt:str, expectedType):
    if type(response:=int(input(prompt))) is not expectedType:
        print("Enter an Integer")
        getUserInput(prompt=prompt, expectedType=expectedType)
    else:
        return response

def main():
    droneDirectory = {}
    locationDirectory = {}
    


    print("                                Welcome to the BOWSER Prototype ---- DEMO ---- ")
    
    
    numberOfDrones = getUserInput(prompt="""
                           Please enter a number of drones to simulate ( Max: 15 )
                           """, expectedType=int)
        
    
    algorithmChoice = getUserInput("""
                            Please Input 1 - 4 to Select Algorithm:
                            ---------------------------------------
                            1: Deep Q Network
                            2: Genetic Algorithm
                            3: Munkres (Hungarian)
                            4: Simulated Annealing
                            """, expectedType=int)+1

    print(f"You have selected {numberOfDrones} Drones")
    ic(algorithmChoice)

    ic("Start Empty Gazebo World")

    print(f"Spawn in {numberOfDrones} Drones")

    for droneID in range(numberOfDrones):
        # addDroneToSimulation(droneID=droneID)
        startingLocation = simulateAddingDrone(droneID=droneID)
        tempDrone = Drone(droneID=droneID, startingLocation=startingLocation, currentLocation=startingLocation)
        droneDirectory[droneID]=(tempDrone)
        locationDirectory[droneID]=(startingLocation)

    ic(droneDirectory)

    ic("Writing Drone Locations into CSV for Algorithms")
    
    writeDictToCSV(locationDirectory,"simulatedDroneLocations.csv")
    
    
    ic("Call Algorithm - Currently Using Template Threat Locations For Sake of Presentation")
    startTime = time()
    # If they chose to run DQN, first it trains a model, then runs it, requiring two algorithm calls
    if algorithmChoice == 2:
        selectAlgorithm(algorithmChoice=algorithmChoice-1)
    results = selectAlgorithm(algorithmChoice=algorithmChoice)
    finishTime = time()
    
    ic("Handle Drones in Simulation According to Algorithm Results") 
    
    ic("Calculate Simulation Leaker Percentage")
    # ic(results)
    print("Returned Output")
    ic(results["Weapon Selection"])
    print(" \n\n            -------------------------------------------------------------------------------------------------")
    print(f"            Simulated Leaker Percentage: XX.XX% | Algorithm Percentage: {results['Leaker Percentage']}%\n\n\n")
    print(f"                                      That Took {finishTime-startTime} Seconds")

if __name__ == "__main__":
    main()



