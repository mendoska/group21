from dataclasses import dataclass, field
from typing import List

@dataclass
class Weapon:
    # ID and name of weapon system
    weaponID: int
    weaponName: str
    
    # location: (x,y,z(eventually)) Defaults to (0, 0, 0)
    
    location: set = (0,0,0)
    
    # How much ammo is left in system
    ammunitionQuantity: int = 10
    # Range at which the weapon system can interact with a target: [(maximum coords), (minimum coords)]
    targetRange:List[set] = field(default_factory=list)
    # Firing rate in seconds
    firingRate:float = 1.0  
    
    
    
    