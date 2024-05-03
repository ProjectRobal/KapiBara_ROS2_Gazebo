#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import Range,Imu
from geometry_msgs.msg import Quaternion

from kapibara_interfaces.srv import Emotions
from kapibara_interfaces.msg import Microphone

from copy import copy

'''

We have sensors topics that will estimate angers 

'''


class EmotionEstimator(Node):

    def __init__(self):
        super().__init__('emotion_estimator')
        
        self.current_ranges=[]
        
        self.declare_parameter('range_threshold', 0.1)
        
        # a list of topics of tof sensors
        self.declare_parameter('tofs', ['/laser_front'])
        
        # a orientation callback
        self.declare_parameter('imu', '/Gazebo/orientation')
        
        self.declare_parameter('mic', '')
        
        self.service = self.create_service(Emotions,'emotions',self.emotions_callback)        
        
        
        # parameter that describe fear distance threshold for laser sensor
        
        self.range_threshold = self.get_parameter('range_threshold').get_parameter_value().double_value
        self.laser_subscriptions=[]
        
        # create N of those subscriptions
        
        tofs = self.get_parameter('tofs').get_parameter_value().string_array_value
        
        for id,tof in enumerate(tofs):
            self.get_logger().info("Creating subscription for TOF sensor at topic: "+tof+" with id "+str(id))
            self.laser_subscriptions.append(self.create_subscription(Range,tof,lambda msg,x=id: self.laser_callback(msg,x),10))
            self.current_ranges.append(0.0)
        
        # Orienataion callback
        
        imu_topic = self.get_parameter('imu').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for IMU sensor at topic: "+imu_topic)
        self.imu_subscripe = self.create_subscription(Imu,imu_topic,self.imu_callback,10)
        
        # Microphone callback use audio model 
        
        mic_topic = self.get_parameter('mic').get_parameter_value().string_value
        
        if len(mic_topic)>0:
            self.get_logger().info("Creating subscription for IMU sensor at topic: "+imu_topic)
            self.mic_subscripe = self.create_subscription(Microphone,mic_topic,self.mic_callback,10)
        
        # IMU, use acclerometer 
        
    
    def laser_callback(self,laser:Range,id):
        self.get_logger().debug("Got range with id "+str(id)+" with range: "+str(laser.range))
        self.current_ranges[id]=laser.range
        
    def imu_callback(self,imu:Imu):
        pass
    
    def mic_callback(self,mic:Microphone):
        pass
    
    def emotions_callback(self, request, response):
        
        self.get_logger().debug("Current ranges kept by program: "+str(self.current_ranges))
        
        response.angry=0.0
        response.fear=( 1.0 - min( ( min( self.current_ranges )/self.range_threshold ),1.0) )*5.0
        response.happiness=0.0
        response.uncertainty=0.0
        response.boredom=0.0
        
        return response


def main(args=None):
    rclpy.init(args=args)

    emotion_estimator = EmotionEstimator()

    rclpy.spin(emotion_estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emotion_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()