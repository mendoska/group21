import subprocess 
from icecream import ic
from random import random
from math import atan2, pi

def runAsyncCommand(command:str) -> str:
    try:
        ic()
        # Run the Zsh command
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True,
            executable='/bin/zsh'  # Specify the Zsh executable
        )
        ic("Process Started")
        return process

        # # Check if the command was successful
        # if result.returncode == 0:
        #     # Return the standard output
        #     return result.stdout.strip()
        # else:
        #     # If the command failed, raise an exception or handle the error
        #     raise subprocess.CalledProcessError(result.returncode, result.stderr)
    except Exception as e:
        # Handle exceptions, if any
        print(f"An error occurred: {e}")
        return None
    
    
    
    

def runSyncCommand(command:str) -> str:
    try:
        ic()
        # Run the Zsh command
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True,
            executable='/bin/zsh'  # Specify the Zsh executable
        )

        # Check if the command was successful
        if result.returncode == 0:
            # Return the standard output
            return result.stdout.strip()
        else:
            # If the command failed, raise an exception or handle the error
            raise subprocess.CalledProcessError(result.returncode, result.stderr)
    except subprocess.TimeoutExpired as e:
        ic(e)
        ic("Drone Created and Process Killed")
        return 1
    except Exception as e:  
        # Handle exceptions, if any
        print(f"An error occurred: {e}")
        return None

# add drone to simulation
""" return created Location """
def addDroneToSimulation(droneID:int, spawnCoordinateX:int, spawnCoordinateY:int) -> None:
    yaw = atan2(spawnCoordinateY,spawnCoordinateX) + pi 
    addDroneCommand = f"""
                        source ../simulation/swarm_ws/ROS2swarm_B/install/setup.zsh &&
                        ROS_DOMAIN_ID=42 ros2 launch launch_gazebo add_robot.launch.py \
                        start_index:={droneID} \
                        gazebo_world:=empty.world \
                        pattern:=drive2OriginPattern \
                        number_robots:=1 \
                        log_level:=info \
                        robot:=waffle_pi \
                        sensor_type:=lidar \
                        version:=2 \
                        x_start:={spawnCoordinateX} \
                        x_dist:=0.5 \
                        y_start:={spawnCoordinateY} \
                        y_dist:=1.0 \
                        yaw:={yaw} \
                        driving_swarm:=False
                     """

    return runAsyncCommand(command=addDroneCommand)
    # if runSyncCommand(command=addDroneCommand) == 1:
    #     ic("Drone Created")
    #     return 1
    # else:
    #     ic(f"Error While Adding Drone {droneID} To Simulation")
    #     return 0

""" return created Location """
def add1DroneToSimulation(droneID:int) -> list:
    # yaw = atan2(spawnCoordinateY,spawnCoordinateX) + pi 
    addDroneCommand = f""" zsh ../simulation/swarm_ws/ROS2swarm_B/add_robots_to_simulation.sh
                          """
    if runSyncCommand(command=addDroneCommand) == 1:
        ic("Drone Created")
        return []
    else:
        ic(f"Error While Adding Drone {droneID} To Simulation")
        return 0

# ping location function
""" returns location   NOT USED """
# def getDroneLocation(droneID: int) -> dict:
#     """
#      Using the drone ID as a marker, We ping the drone's transform topic node, 
#      taking the first 17 lines, which make up the first message, and transform it from 
#      its yaml formatting into a dictionary using the yaml.safe_load() function.
#      Then we path to the location which takes the form of a dictionary structured
#      as such: {'x': x-coordiate, 'y': y-coordiate, 'z': z-coordiate}
#      NOTE: Only the x and y coordinates are necessary for this prototype.
     
#     Returns:
#         Dictionary: {'x': x-coordiate, 'y': y-coordiate, 'z': z-coordiate} of Drone
#     """
   
#     locationCommand = f"ROS_DOMAIN_ID=42 ros2 topic echo /robot_{droneID}/tf | head -n 17"
#     # if the command response is not none (which implies an error while calling the subprocess)
#     # then the variable is set to command response 
#     if (commandResponse:=runSyncCommand(command=locationCommand)) is not None:
#         # the full yaml response is converted to a dictionary and passed into the locationDictionary
#         locationDictionary = safe_load(commandResponse)
#         ic(locationDictionary)
#         # then the locationDicitonary is set to the specific transformation dictionary containing 
#         # the x, y, and z coordiantes of the drone 
#         locationDictionary = locationDictionary['transforms'][0]['transform']['translation']
#         ic(locationDictionary)  
#         return locationDictionary
#     else:
#         ic(f"Error While Echoing Robot {droneID}'s Transformation Topic")
#         return 0


def issueStartCommand() -> None:
    startCommand = "zsh ../simulation/swarm_ws/ROS2swarm_B/start_command.sh"
    runSyncCommand(command=startCommand)
    return


# remove drone from simulation
""" return 'destroyed' location  """
def destroyDrone(droneID: int) -> list:
    """
    set velocity = 0
    location
    remove


    return [locationList, confirmation]
    """
    delRobotModel = f"""ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity '{{name: "robot_{droneID}"}}' """
    if (commandResponse:=runSyncCommand(command=delRobotModel)) is not None: 
        status_message = commandResponse.split("status_message")[1] # Isolates just the status message portion
        status_message = status_message[2:len(status_message)-2].replace("[","").replace("]","").replace("_"," ") # more formatting to make it prettier
        return(status_message)
    pass

    
# set Drone Velocity Function
""" set cmd_vel for drone """
def setDroneVelocity(droneID:int, xVelocity:float = 0, yVelocity:float = 0, zVelocity:float = 0) -> str:
    # FROM DRIVE_PATTERN.PY IN ROS2SWARM
     # command to publish the message in the terminal by hand
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        # linear: {x: 0.26, y: 0.0, z: 0.0},
        # angular: {x: 0.0, y: 0.0, z: 0.0}
        # }"
    
    
    pass


# start drone
""" return confirmation """
def startDrone(droneID:int) -> str:
    pass


# Start World Simulation
def startEmptyGazeboWorldSimulation(totalNumberOfDrones:int) -> subprocess.Popen:
    startWorldCommand = f"""
            cd ../simulation/swarm_ws/ROS2swarm_B &&
            colcon build --symlink-install &&
            source ./install/setup.zsh &&
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
    return runAsyncCommand(command=startWorldCommand)

    
 
 
    