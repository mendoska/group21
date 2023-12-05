
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from icecream import ic


class DroneController(Node):
    def __init__(self, droneID:int):
        super().__init__(f'drone_controller_{droneID}')
        self.id=droneID
        self.publisher_ = self.create_publisher(Twist, f'/robot_{self.id}/cmd_vel', 10)  # Update with the correct topic
        self.odometry_subscription_ = self.create_subscription(Odometry, f'/robot_{self.id}/odom', self.odometry_callback, 10)  # Update with the correct topic
        self.odometry_data = None
        self.timer = self.create_timer(1.0, self.timerCallback)
        self.target_position = {'x': 0.0, 'y': 0.0, 'z':0.0}
        
    def getID(self):
        return self.id
    
    def odometry_callback(self, msg):
        # Store the odometry data for later use
        self.odometry_data = msg

    def timerCallback(self):
      if self.is_at_target_position is False:
        self.move_to_target_position(location=self.get_current_position())
      


    def get_current_position(self):
        # Extract and return the current position from the stored odometry data
        if self.odometry_data:
            current_position = self.odometry_data.pose.pose.position
            return {'x': current_position.x, 'y': current_position.y, 'z': current_position.z}
        else:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
    def move_to_target_position(self, location:dict):
        
         #  Calculate differences between current and target positions
        dx = 0 - location['x']
        dy = 0 - location['y']
        dz = 0 - location['z']

        # Calculate linear and angular velocities
        linear_velocity = math.sqrt(dx**2 + dy**2)
        angular_velocity = math.atan2(dy, dx)
        


        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.publisher_.publish(twist_msg)
        
    


    def is_at_target_position(self, threshold=0.5):
        current_position = self.get_current_position()
        return abs(current_position['x'] - self.target_position['x']) < threshold and \
               abs(current_position['y'] - self.target_position['y']) < threshold
               
        