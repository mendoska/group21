from yaml import safe_load, dump
from icecream import ic

# filePath = '../simulation/swarm_ws/ROS2swarm_B/src/ros2swarm/config/waffle_pi/movement_pattern/basic/drive2OriginPattern.yaml'
# with open(file=filePath,mode='r') as file:
#     data = safe_load(file)

# ic(data["/**"]["ros__parameters"]["speed"])
# # data["/**"]["ros__parameters"]["leaker_range"] = leaker_range

# # Write back to YAML file
# with open(file=filePath, mode='w') as file:
#     dump(data, file)
    
        
        
lr = 0.1
string = "[drive2OriginPattern-4] [INFO] [1711065979.998261696] [robot_1.drive2OriginPattern]: Leaker Range is: 0.0"       
ic(f"Leaker Range is: {lr}" in string) 