from icecream import ic
from fakeSimulationFunctions import simulateAddingDrone, writeDictToCSV
from Models.Drone import Drone
from bridge import selectAlgorithm


def getUserInput(prompt:str, expectedType):
    if type(response:=int(input(prompt))) is not expectedType:
        print("Enter an Integer")
        getUserInput(prompt=prompt, expectedType=expectedType)
    else:
        return response

def main():
    droneDirectory = {}
    locationDirectory = {}
    


    ic("                   Welcome to the BOWSER Prototype")
    
    
    numberOfDrones = getUserInput(prompt="""
                           Please enter a number of drones to simulate ( Max: 20 )
                           """, expectedType=int)
        
    
    algorithmChoice = getUserInput("""
                            Please Input 1 - 4 to Select Algorithm:
                            ---------------------------------------
                            1: Deep Q Network
                            2: Genetic Algorithm
                            3: Munkres (Hungarian)
                            4: Simulated Annealing
                            """, expectedType=int)

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
    results = selectAlgorithm(algorithmChoice=algorithmChoice)
    
    ic("Calculate Simulation Leaker Percentage")
    print("Returned Output")
    ic(results["Weapon Selection"])
    print(" \n\n            -------------------------------------------------------------------------------------------------")
    print(f"            Simulated Leaker Percentage: XX.XX% | Algorithm Percentage: {results['Leaker Percentage']}%\n\n\n")

if __name__ == "__main__":
    main()



