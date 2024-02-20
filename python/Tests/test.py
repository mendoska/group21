import subprocess
import shlex
import signal
import time
from icecream import ic

ros2_node_list_command = "ros2 node list"
nodes = subprocess.run(shlex.split(ros2_node_list_command), capture_output=True, text=True).stdout.splitlines()
# Use ros2 node kill to terminate each node
ic(nodes)
# for node in nodes:
#     ros2_node_kill_command = f"ros2 node kill {node}"
#     subprocess.run(shlex.split(ros2_node_kill_command))
# # Wait for a short duration to allow nodes to terminate
# time.sleep(2)
# Use pkill to terminate any remaining ROS-related processes
pkill_ros_command = "pkill -f 'ros'"
subprocess.run(shlex.split(pkill_ros_command))
    