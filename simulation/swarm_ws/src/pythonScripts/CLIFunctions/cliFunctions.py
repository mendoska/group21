import subprocess 
from yaml import safe_load
from icecream import ic
from random import random
from math import atan2,pi

async def runAsyncCommand(command:str) -> str:
    try:
        ic()
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

def runSyncCommand(command:str, timeOut:float=None) -> str:
    try:
        ic()
        # Run the Zsh command
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True,
            timeout=timeOut,
            executable='/bin/bash'  # Specify the Zsh executable
        )

        # Check if the command was successful
        if result.returncode == 0:
            # Return the standard output
            return result.stdout.strip()
        else:
            # If the command failed, raise an exception or handle the error
            raise subprocess.CalledProcessError(result.returncode, result.stderr)
    except subprocess.TimeoutExpired:
        ic("Drone Created and Process Killed")
        return 1
    except Exception as e:  
        # Handle exceptions, if any
        print(f"An error occurred: {e}")
        return None

# add drone to simulation
""" return created Location """
def addDroneToSimulation(droneID:int) -> list:
    # calculate random x and y between a and b
    x=-20+droneID
    y=-20+droneID
    # using a little bit of trig, I am able to calculate the angle necessary to make the drone face the origin, 
    # that way, when giving the drone a velocity, it will travel directly over the origin line.
    yaw = (atan2(y, x)+pi)
    addDroneCommand = f"""ROS_DOMAIN_ID=42 ros2 launch launch_gazebo add_robot.launch.py \
                          start_index:={droneID} \
                          gazebo_world:=arena_large.world \
                          pattern:=drive_pattern \
                          number_robots:=1 \
                          log_level:=info \
                          robot:=burger \
                          sensor_type:=lidar \
                          version:=2 \
                          x_start:={x} \
                          x_dist:=0.0 \
                          y_start:={y} \
                          y_dist:=0.0 \
                          yaw:={yaw} \
                          driving_swarm:=False \
                          """
    if runSyncCommand(command=addDroneCommand, timeOut=2) == 1:
        # ic("Drone Created")
        return []
    else:
        ic(f"Error While Adding Drone {droneID} To Simulation")
        return 0

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
   
    locationCommand = f"ROS_DOMAIN_ID=42 ros2 topic echo /robot_{droneID}/odom | head -n 17"
    # if the command response is not none (which implies an error while calling the subprocess)
    # then the variable is set to command response 
    if (commandResponse:=runSyncCommand(command=locationCommand)) is not None:
        # the full yaml response is converted to a dictionary and passed into the locationDictionary
        ic(commandResponse)
        locationDictionary = safe_load(commandResponse)
        ic(locationDictionary)
        # then the locationDicitonary is set to the specific transformation dictionary containing 
        # the x, y, and z coordiantes of the drone 
        locationDictionary = locationDictionary['pose']['pose']['position']
        ic(locationDictionary)  
        return locationDictionary
    else:
        ic(f"Error While Echoing Robot {droneID}'s Transformation Topic")
        return 0

def removeDroneModel(droneID:int) -> int:
    removeModelCommand = f"gz model -m robot_{droneID} -d"
    if (response := runSyncCommand(removeModelCommand)) is not None:
        ic(f"Drone Model Removed, Robot {droneID} Nodes Still Exist")
        return 1
    else:
        ic("Drone {droneID} Model Could Not Be Removed")
        return 0
    
    
    
# remove drone from simulation
""" return 'destroyed' location  """
def destroyDrone(droneID: int) -> list:
    """
    set velocity = 0
    
    location
    remove


    return [locationList, confirmation]
    """
    set
    
    
    
    pass

    
# set Drone Velocity Function
""" set cmd_vel for drone """
def setDroneVelocity(droneID:int, xVelocity:float = 0, yVelocity:float = 0, zVelocity:float = 0) -> str:
    # FROM DRIVE_PATTERN.PY IN ROS2SWARM
     # command to publish the message in the terminal by hand
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        # linear: {x: 0.26, y: 0.0, z: 0.0},
        # angular: {x: 0.0, y: 0.0, z: 0.0} }"
    velocityDict = {"linear": {"x": xVelocity, "y": yVelocity, "z": zVelocity}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
    driveCommand = f'ros2 topic pub --once /robot_{droneID}/cmd_vel geometry_msgs/msg/Twist "{velocityDict}"'
    # if the command response is not none (which implies an error while calling the subprocess)
    # then the variable is set to command response 
    if (commandResponse:=runSyncCommand(command=driveCommand)) is not None:
        # the full yaml response is converted to a dictionary and passed into the locationDictionary
        # response = safe_load(commandResponse)
        ic(commandResponse)
        # then the locationDicitonary is set to the specific transformation dictionary containing 
        # the x, y, and z coordiantes of the drone 
        
        return 1
    else:
        ic(f"Error While Echoing Robot {droneID}'s Transformation Topic")
        return 0
    
    pass


# start drone NOT NEEDED RIGHT NOW
""" return confirmation """
def startDrone(droneID:int) -> str:
    pass


# Start World Simulation
async def startEmptyGazeboWorldSimulation(totalNumberOfDrones:int) -> None:
    startWorldCommand = f"""
            ROS_DOMAIN_ID=42 ros2 launch launch_gazebo create_enviroment.launch.py \
            gazebo_world:=empty.world \
            pattern:=drive_pattern \
            number_robots:=0 \
            total_robots:={totalNumberOfDrones} \
            log_level:=info \
            robot:=waffle_pi \
            sensor_type:=lidar \
            x_start:=20.0 \
            x_dist:=0.0 \
            y_start:=0.0 \
            y_dist:=1.0 \
            driving_swarm:=False \
            logging:=False 
            """
 
    if (commandResponse:= await runAsyncCommand(command=startWorldCommand)) is not None:
        ic(commandResponse)
    else:
        ic(f"Error While Creating World With {totalNumberOfDrones} Drones")
        return 0
 
 
    