'''

 A module for controling KapiBara robot in step mode.
 
 It takes action to move in one of specified direction: 
    - left
    - right
    - top
    - bottom
    
 It also keep tracks on distance sensors, orientation and position in simulation
'''

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from rosgraph_msgs.msg import Clock

class KapiBaraStepAgent:
    
    '''
        Actions for agents: up , down , left , right
        in form of pair ( linear speed, angular speed)
    '''
    _actions = [(-1.0,0.0),(1.0,0.0),(0.0,1.0),(0.0,-1.0)]
    
    def clock_step_counter(self,clock):
        self._step_count += 1
        
        self._node.get_logger().info(f"Got frame: {self._step_count}")
    
    def tof_callback(self,id,tof_msg:Range):
        self._observations[id] = tof_msg.range
        
        self._node.get_logger().info(f"Got range id: {id}")
        
    def orientaion_callback(self,orientaion:Quaternion):
        self._observations[4] = orientaion.x
        self._observations[5] = orientaion.y
        self._observations[6] = orientaion.z
        self._observations[7] = orientaion.w
        
        self._node.get_logger().info("Got orientation!")
    
    def odometry_callback(self,odometry:Odometry):
        position = odometry.pose.pose.position
        
        self._observations[8] = position.x
        self._observations[9] = position.y
        self._observations[10] = position.z
        
        self._node.get_logger().info("Got odometry!")
        
    
    def __init__(self,parent_node:Node, max_linear_speed:float=None, max_angular_speed:float=None) -> None:
        
        self._node = parent_node
        
        self._observations = np.zeros(11,dtype=np.float32)
        
        if max_angular_speed is None:
            self._max_angular_speed = 3.0
        else:
            self._max_angular_speed = max_angular_speed
            
        if max_linear_speed is None:
            self._max_linear_speed = 1.0
        else:
            self._max_linear_speed = max_linear_speed
        
        # creates subscription for laser sensors
        
        self._tof_sub = []
        
        self._tof_sub.append(self._node.create_subscription(Range,"/Gazebo/front_left",lambda msg: self.tof_callback(0,msg),10))
        self._tof_sub.append(self._node.create_subscription(Range,"/Gazebo/front_right",lambda msg: self.tof_callback(1,msg),10))
        self._tof_sub.append(self._node.create_subscription(Range,"/Gazebo/side_left",lambda msg: self.tof_callback(2,msg),10))
        self._tof_sub.append(self._node.create_subscription(Range,"/Gazebo/side_right",lambda msg: self.tof_callback(3,msg),10))
                
        # creates subscription for orientation
        
        self._orientaion_sub = self._node.create_subscription(Quaternion,"/Gazebo/orientation",self.orientaion_callback,10)
        
        # creates subcription for position
        
        self._odometry_sub = self._node.create_subscription(Odometry,"/motors/odom",self.odometry_callback,10)
        
        # creates publisher for ros2 control twist
        
        self.twist_output = self._node.create_publisher(Twist, "/motors/cmd_vel_unstamped", 10)
        
        self._step_count = 0
        
        self._step_counter_qos =  rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                                          depth=10)
        
        # step counter
        self._step_counter = self._node.create_subscription(Clock,"/clock",self.clock_step_counter,qos_profile=self._step_counter_qos)

        
    def move(self,direction):
        
        twist = Twist()
        
        direction = np.clip(direction,0,3)
        
        action = self._actions[direction]
        
        twist.angular.z = action[1] * self._max_angular_speed
        twist.linear.x = action[0] * self._max_linear_speed
        
        self.twist_output.publish(twist)
                
        #rclpy.spin_once(self._node)
        
    def wait_for_steps(self):
        self._step_count = 0
        while self._step_count < 10:
            self._node.get_logger().info("Waiting for clock!")
            rclpy.spin_once(self._node)
            
    def get_observations(self)->np.ndarray:
        
        rclpy.spin_once(self._node)
        
        return self._observations