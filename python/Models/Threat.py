from dataclasses import dataclass, field
from typing import List

@dataclass
class Threat:
    # ID of drone, drones will be created robot_{ID}
    threatID: int
    
    # Either Alive (Spawned in the environment and moving),  
    # or Dead (despawned or destroyed)
    currentStatus: str = "Dead"
    
    
    # Leaker Range: The distance at which a threat will be seen as a "leaker"
    leakerRange: float = 0.5
    
    # Threat Speed: The speed at which a threat will move within the environment (m/s)
    speed: float = 3.0
    
    # location list: [x,y,z(eventually)]
    startingLocation: List[int] = field(default_factory=list)
    destroyedLocation: List[int] = field(default_factory=list)  # Unused Currently
    currentLocation: List[int]  = field(default_factory=list)   # Unused Currently
    
    
    
    

 