import subprocess 
from yaml import safe_load

from icecream import ic

def run_Command(command:str) -> str:
    try:
        # Run the Zsh command
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True,
            executable='/bin/bash'  # Specify the Zsh executable
        )

        # Check if the command was successful
        if result.returncode == 0:
            # Return the standard output
            return result.stdout.strip()
        else:
            # If the command failed, raise an exception or handle the error
            raise subprocess.CalledProcessError(result.returncode, result.stderr)
    except Exception as e:
        # Handle exceptions, if any
        print(f"An error occurred: {e}")
        return None



# ping location function
""" returns location """
def getDroneLocation(droneID: int) -> dict:
    """
     Using the drone ID as a marker, We ping the drone's transform topic node, 
     taking the first 17 lines, which make up the first message, and transform it from 
     its yaml formatting into a dictionary using the yaml.safe_load() function.
     Then we path to the location which takes the form of a dictionary structured
     as such: {'x': x-coordiate, 'y': y-coordiate, 'z': z-coordiate}
     NOTE: Only the x and y coordinates are necessary for this prototype.
     
    Returns:
        Dictionary: {'x': x-coordiate, 'y': y-coordiate, 'z': z-coordiate} of Drone
    """
   
    locationCommand = f"ROS_DOMAIN_ID=42 ros2 topic echo /robot_{droneID}/tf | head -n 17"
    # if the command response is not none (which implies an error while calling the subprocess)
    # then the variable is set to command response 
    if (commandResponse:=run_Command(command=locationCommand)) is not None:
        # the full yaml response is converted to a dictionary and passed into the locationDictionary
        locationDictionary = safe_load(commandResponse)
        ic(locationDictionary)
        # then the locationDicitonary is set to the specific transformation dictionary containing 
        # the x, y, and z coordiantes of the drone 
        locationDictionary = locationDictionary['transforms'][0]['transform']['translation']
        ic(locationDictionary)  
        return locationDictionary
    else:
        ic(f"Error While Echoing Robot {droneID}'s Transformation Topic")
        return 0

# async  add drone to simulation
""" 
return created Location
"""


# remove drone from simulation
""" 
return 'destroyed' location  

set velocity = 0
location
remove


return location, confirmation
"""

    
# set Drone Velocity Function
""" 
set cmd_vel for drone
"""

# start drone
"""

return confirmation
"""
    