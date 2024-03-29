from dataclasses import dataclass, field
from typing import List

@dataclass
class Drone:
    # ID of drone, drones will be created robot_{ID}
    droneID: int
    
    # Either Ready (Spawned in and Stationary), Driving, 
    # or Dead (despawned or destroyed)
    currentStatus: str = "Dead"
    
    # location list: [x,y,z(eventually)]
    startingLocation: List[int] = field(default_factory=list)
    destroyedLocation: List[int] = field(default_factory=list)
    currentLocation: List[int]  = field(default_factory=list)
    
    
    
    

 