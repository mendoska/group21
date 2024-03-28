from Models.Weapon import Weapon
from icecream import ic  
from csv import reader



w1 = Weapon(weaponID= 0, 
            weaponName= "MouseKaTool",
            location=[0,0,0]
            )
threatFileLocation = "dataFiles/weapon_data.csv"

def initializeWeaponsFromFile(threat_file)-> list:
    with open(threat_file, 'r') as file:
        weapons = []
        readerObj = reader(file)
        for weaponID, threatEntry in enumerate(list(readerObj)):
            weaponPlacement = Weapon(weaponID=weaponID,
                                     weaponName=threatEntry[0],
                                     location=[threatEntry[1], threatEntry[2], threatEntry[3]],
                                     ammunitionQuantity=threatEntry[5],
                                     firingRate=threatEntry[6]
                                    )
            weapons.append(weaponPlacement)
        
        return weapons
    
x = initializeWeaponsFromFile(threat_file=threatFileLocation)

ic(x)
    
    
