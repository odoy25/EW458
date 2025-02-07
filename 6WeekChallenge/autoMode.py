import pygame 
import time
import threading as th
import roslibpy

class autoMode():
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name
        # define color and noise for idle mode
        self.color = {'yellow': [{'red': 255, 'green': 255, 'blue': 0}] * 6}
        self.noise = 7     
           
        # create topics for publishing
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.vel_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/Twist')
    
    def publish_flashing_yellow(self):
        # publish yellow color
        led_msg = {'leds': self.color['yellow'], 'override_system': True}
        if self.ros_node.is_connected:
            self.led_pub.publish(roslibpy.Message(led_msg))
    
    def make_auto_noise(self):
        # publish noise
        noise_msg = {'notes': [self.noise]}
        if self.ros_node.is_connected:
            self.noise_pub.publish(roslibpy.Message(noise_msg))
    
    def mow():
        pass