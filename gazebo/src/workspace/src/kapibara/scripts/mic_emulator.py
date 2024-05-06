#!/usr/bin/env python3

import pyaudio
import numpy as np

import rclpy
from rclpy.node import Node

from kapibara_interfaces.msg import Microphone as Mic

CHUNK_SIZE=512

class Microphone:
    def __init__(self,chunk=CHUNK_SIZE,format=pyaudio.paInt32,channels=1,rate=44100,id=0,callback=None):

        '''init input audio device'''
        self.audio=pyaudio.PyAudio()
        self.open_stream(format,channels,rate,chunk,id)
        self.callback=callback
        
    def callback(self,in_data, frame_count, time_info, status):
        # If len(data) is less than requested frame_count, PyAudio automatically
        # assumes the stream is finished, and the stream stops.
        
        if self.callback is not None:
            self.callback(np.frombuffer(in_data, dtype=np.int32))
        
        return (in_data, pyaudio.paContinue)


    '''Reopen a stream with different settings'''
    def open_stream(self,format,channels,rate,chunk,id=0):

        self.close_stream()

        self.format = format
        self.channels = channels
        self.rate = rate
        self.chunk = chunk
        self.id = id

        self.stream=self.audio.open(format=format, channels=channels,rate=rate,frames_per_buffer=chunk,input=True,input_device_index=id,stream_callback=self.callback)
        self.chunk=chunk

    '''Get a chunk then check if it has a voice , if it is true it return a array with chunk else it returns None '''
    def get(self):
        frame=self.stream.read(self.chunk,exception_on_overflow = False)

        return np.frombuffer(frame,dtype=np.int16)

    '''Close current stream'''
    def close_stream(self):
        if hasattr(self,'stream'):
            self.stream.stop_stream()
            self.stream.close()

    '''Close microphone completely'''
    def close(self):
        self.close_stream()
        self.audio.terminate()
        

class MicEmulator(Node):

    def __init__(self):
        super().__init__('MicEmulator')
        
        self.declare_parameter('mic_topic','/mic')
        self.declare_parameter('rate',44100)
        self.declare_parameter('dev_id',1)
        
        rate:int=self.get_parameter('rate').get_parameter_value().integer_value
        id:int=self.get_parameter('dev_id').get_parameter_value().integer_value
        
        self.get_logger().info("Starting audio input device with id: "+str(id)+" and sample rate "+str(rate))
        self.microphone=Microphone(rate=rate,callback=self.audio_callback,id=id)
        
        self.get_logger().info("Creating publisher for microphone")
        self.audio_publisher=self.create_publisher(Mic, self.get_parameter('mic_topic').get_parameter_value().string_value, 10)
        
    def audio_callback(self,data:np.ndarray):
        
        mic:Mic=Mic()
        
        mic.buffor_size=CHUNK_SIZE
        mic.channel1=data
        mic.channel2=data
        
        self.audio_publisher.publish(mic)
        self.get_logger().info("Audio published")
        

def main(args=None):
    rclpy.init(args=args)

    sensors = MicEmulator()

    rclpy.spin(sensors)

    sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()