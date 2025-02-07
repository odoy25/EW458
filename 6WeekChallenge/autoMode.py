import pygame 
import time
import threading as th
import roslibpy
from idleMode import idleMode

class autoMode():
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name
           
        # create topics for publishing
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.vel_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/Twist')

        imu_topic = roslibpy.Topic(self.ros_node, f'{robot_name}/imu', 'sensor_msgs/Imu')

    
    def activate(self):
        # initialize joystick
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # perform auto-specific actions
        self.publish_flashing_yellow()
        self.make_auto_noise()

    def cleanup(self):
        # prepare to exit mode
        pygame.quit()
        print("Exiting auto mode.")

    def publish_flashing_yellow(self):
        # publish distinct auto color (flashing yellow)
        color = {'yellow': [{'red': 255, 'green': 255, 'blue': 0}] * 6}
        if self.ros_node.is_connected:
            led_msg = {'leds': color['yellow'], 'override_system': True}
            self.led_pub.publish(roslibpy.Message(led_msg))
    
    def make_auto_noise(self):
        # publish distinct auto noise
        if self.ros_node.is_connected:
            noise_msg = {
                'notes': [
                    {'frequency': 680, 'max_runtime': {'sec': 0, 'nanosec': 100000000}},
                    {'frequency': 370, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
                    {'frequency': 680, 'max_runtime': {'sec': 0, 'nanosec': 100000000}},
                    {'frequency': 370, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
                ]
            }
            self.noise_pub.publish(roslibpy.Message(noise_msg))
    
    def mow():
        pass
        # reset odometry
        # drive straight
            # while driving, use sensor
                # if sensor detects odd obstacle:
                    # turn left
                    # drive straight x amount
                    # turn left again
                # if sensor detects even obstacle:
                    # turn right
                    # drive straight x amount
                    # turn right again
            # sense that we are done driving
        
        # go to idle mode
            