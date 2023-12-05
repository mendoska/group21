from random import randint
from icecream import ic




def simulateAddingDrone(droneID:int) -> dict:
    randomRange = (10,20)
    
    startingX = randint(a=randomRange[0],b=randomRange[1])
    startingy = randint(a=randomRange[0],b=randomRange[1])
    startingZ = 0
    
    print(f"Spawning Drone at ({startingX},{startingy},{startingZ})")
    startingLocation = {"x":startingX,"y":startingy,"z":startingZ}
    
    
    return startingLocation




import csv

def writeDictToCSV(dictionary, csv_filename):
    """
    Write a Python dictionary to a CSV file.

    Parameters:
        dictionary (dict): The dictionary to be written to the CSV file.
        csv_filename (str): The name of the CSV file.

    Returns:
        None
    """
    with open(csv_filename, 'w', newline='') as csvfile:
        fieldnames = ['droneID', 'x', 'y', 'z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write the header
        writer.writeheader()

        # Write each key-value pair as a separate row
        for key, values in dictionary.items():
            writer.writerow({'droneID': key, 'x': values.get('x', ''), 'y': values.get('y', ''), 'z': values.get('z', '')})



# Example usage:

