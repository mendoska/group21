import subprocess
import shlex
import signal
import time
from icecream import ic

ros2_node_list_command = "ros2 node list"
nodes = subprocess.run(shlex.split(ros2_node_list_command), capture_output=True, text=True).stdout.splitlines()
# Use ros2 node kill to terminate each node
ic(nodes)
# Use pkill to terminate any remaining ROS-related processes
pkill_ros_command = "pkill -f 'ros'"
subprocess.run(shlex.split(pkill_ros_command))
    