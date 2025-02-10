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

        self.odom_sub = roslibpy.Topic(self.ros_node, f'{robot_name}/odom', 'nav_msgs/Odometry')

        # intialize for velocity control
        self.velocity_lock = th.Lock()
        self.running = False
        self.vel_thread = None

    def activate(self):
        # initialize joystick
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # perform auto-specific actions
        self.publish_flashing_yellow()
        self.make_auto_noise()
        self.mow()

    def cleanup(self):
        # prepare to exit mode
        self.running = False 
        if self.vel_thread is not None:
            self.vel_thread.join() # stop the velocity thread
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

    def velocity_publisher(self):
        while self.running:
            self.drive_stright()
            if self.ros_node.is_connected:
                self.vel_pub.publish(roslibpy.Message(self.current_velocity))
            time.sleep(0.05)  # publish every 50ms

    def drive_straight(self, meters):
        with self.velocity_lock:
            self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}} # for meters specified

    def turn_left(self):
        self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.3}}

    def turn_right(self):
        self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': -0.3}}

    def sense_head_on(self):
        self.object_detected = False
        # get sensor data (subscribe to sensor data)

    def mow(self):
        # get position and then reset odom
        self.odom_sub.subscribe(lambda msg: print(msg['pose']['pose']['position']))
        reset_odom = roslibpy.Service(self.ros_node, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        reset_odom.call(roslibpy.ServiceRequest())

        turn_counter = 1

        # while x position has not reached the far left lateral limit of the boundary:
            self.drive_straight(2.286) # drive at least 2.286 m
            self.sense_head_on() # need to thread with drive_straight so they run at the same time
            
            turn_counter += 1
            if self.object_detected:
                if turn_counter % 2 == 0:
                    self.turn_right()
                    self.drive_straight(0.2) # drive straight 0.2 m
                    self.turn_right()
                else:
                    self.turn_left()
                    self.drive_straight(0.2) # drive straight 0.2 m
                    self.turn_left()            
        # if our x position exceeds 4.572 m:          
            # go to idle mode
            