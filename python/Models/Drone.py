from dataclasses import dataclass, field

@dataclass
class Drone:
    # ID of drone, drones will be created robot_{ID}
    droneID: int
    
    # Either Ready (Spawned in and Stationary), Driving, 
    # or Dead (despawned or destroyed)
    currentStatus: str = "Dead"
    
    # location list: [x,y,z(eventually)]
    startingLocation: list[int] = field(default_factory=list)
    destroyedLocation: list[int] = field(default_factory=list)
    currentLocation: list[int]  = field(default_factory=list)
    
    
    
    

 