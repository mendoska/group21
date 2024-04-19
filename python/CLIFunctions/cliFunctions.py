import subprocess 
from icecream import ic
from random import random
from math import atan2, pi
from shlex import split

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
            executable='/bin/bash'  # Specify the Zsh executable
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
            executable='/bin/bash'  # Specify the Zsh executable
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
        ic("Threat Created and Process Killed")
        return 1
    except Exception as e:  
        # Handle exceptions, if any
        print(f"An error occurred: {e}")
        return None

# add drone to simulation
""" return created Location """
def addThreatToSimulation(threatID:int, spawnCoordinateX:int, spawnCoordinateY:int) -> None:
    yaw = atan2(spawnCoordinateY,spawnCoordinateX) + pi 
    addDroneCommand = f"""
                        source ../simulation/swarm_ws/ROS2swarm_B/install/setup.zsh &&
                        ROS_DOMAIN_ID=42 ros2 launch launch_gazebo add_robot.launch.py \
                        start_index:={threatID} \
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







def issueStartCommand() -> None:
    startCommand = "zsh ../simulation/swarm_ws/ROS2swarm_B/start_command.sh"
    runSyncCommand(command=startCommand)
    return


# remove drone from simulation
""" return 'destroyed' location  """
def destroyThreat(threatID: int) -> list:
    """
    set velocity = 0
    location
    remove


    return [locationList, confirmation]
    """
    delRobotModel = f"""ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity '{{name: "robot_{threatID}"}}' """
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

    
def stopWorldSimulation() -> None:
    pkill_ros_command = "pkill -f 'ros'"
    subprocess.run(split(pkill_ros_command))
    return
 
    