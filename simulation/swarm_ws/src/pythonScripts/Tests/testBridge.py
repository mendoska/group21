import subprocess 
# import asynchio
from icecream import ic
from yaml import safe_load
import time



""" JUST A TEST FILE FOR PLANNING

    NOT USED FOR ANYTHING
"""










def run_zsh_command(command):
    try:
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
    except Exception as e:
        # Handle exceptions, if any
        print(f"An error occurred: {e}")
        return None

x = time.time()

# example robot 0 transform parse
zsh_command = 'ROS_DOMAIN_ID=42 ros2 topic echo /robot_0/tf | head -n 17'
output = run_zsh_command(zsh_command)

if output is not None:
    print(f"Zsh command output: {(output)}")
    # parse yaml string into a dictionary
    d = safe_load(output)
    # icecream debug output to better visualize response
    ic(d)
    # prints the dictionary containing the x, y, and z coordinates while also 
    # initializing l as the dictionary containing the positional dictionary
    print(l:=d['transforms'][0]['transform']['translation'])
    # print(l)
    
    # ic(output)
else:
    print("Failed to run command.")
    
y = time.time()

print(y-x)
 
# for x in range(3):
    
#     addRobotCommad = f'ROS_DOMAIN_ID=42 ros2 launch launch_gazebo add_robot.launch.py \
#      start_index:={x} \
#      gazebo_world:=arena_large.world \
#      pattern:=drive_pattern \
#      number_robots:=1 \
#      log_level:=info \
#      robot:=burger \
#      sensor_type:=lidar \
#      version:=2 \
#      x_start:={1.0+x} \
#      x_dist:=0.5 \
#      y_start:=0.0 \
#      y_dist:=1.0 \
#      driving_swarm:=False '
#     out = run_zsh_command(addRobotCommad)
#     print(f"run {x}")
#     ic(out)
    
     


""" HANDLE DRONES FROM ALGORITHM RESULT

DRONE DICTIONARY KEY = DRONEID VALUE=DRONECLASSOBJECT
drones={droneClass}

for droneID in {dictOfDrones}:
    if weapon:=dict{droneID} is ready:
        weapon.count =- 1
        confirmed = drone[droneID].destroyDrone()
        if confirmed:
            remove droneID from dict
        else:
            leakers =+ 1

"""






""" 
async

__________

Drone Class
--------------

stores:
    drone id
    current Status (ready, driving, dead)
    position {x,y,z}
    spawn location []
    destroyed location []
    
    
functions:
    
    createDrone() ->
    
    getLocation(droneID) ->
    destroyDrone -> [confirmationStatus,location]
    setDestroyedLocation  ->
    
    
    
    
"""



""" CREATE DRONE
generate random number for x and y outside of set radius
add drone at that x and y

set velocity of drone to -x, -y


"""






""" Need a way to initialize multiple drones asyncronusly """





"""
select algorithm after drones have started
"""








