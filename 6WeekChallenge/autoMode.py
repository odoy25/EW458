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

        # subscribe to odometry for position feedback
        self.odom_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')

        # subscribe to sensor for obstacle detection
        self.obstacle_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/sensor_data', 'sensor_msgs/LaserScan')  

        # robot state
        self.x_position = 0.0  # initial x position of the robot
        self.object_detected = False  # flag to track if an obstacle is detected
        
        # LED blinking event
        self.blinking_event = th.Event()
        self.blink_thread = None
        self.velocity_thread = None

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
        self.blinking_event.clear()  
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join() # stop the blinking on the LED
        if self.velocity_thread and self.velocity_thread.is_alive():
            self.velocity_thread.join() # stop the velocity thread
        pygame.quit()
        print("Exiting auto mode.")

    def publish_flashing_yellow(self):
        def blink_leds():
            color = {'yellow': [{'red': 255, 'green': 255, 'blue': 0}] * 6}
            while self.blinking_event.is_set():
                led_msg = {'leds': color['yellow'], 'override_system': True}
                if self.ros_node.is_connected:
                    self.led_pub.publish(roslibpy.Message(led_msg))
                time.sleep(1)  # blink on
                led_msg = {'leds': color['yellow'], 'override_system': False}
                if self.ros_node.is_connected:
                    self.led_pub.publish(roslibpy.Message(led_msg))
                time.sleep(1)  # blink off
        
        # start the blinking thread
        self.blinking_event.set()
        self.blink_thread = th.Thread(target=blink_leds, daemon=True)
        self.blink_thread.start()

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

    def move_robot(self):
        # continuously move the robot at a fixed velocity
        while self.object_detected is False:
            self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
            if self.ros_node.is_connected:
                self.vel_pub.publish(roslibpy.Message(self.current_velocity))  # start moving forward
            time.sleep(0.1)

    def stop(self):
        # stop the robot's movement by setting the velocity to zero
        self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        if self.ros_node.is_connected:
            self.vel_pub.publish(roslibpy.Message(self.current_velocity))

    def turn_left(self):
        with self.velocity_lock:
            self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.3}}

    def turn_right(self):
        with self.velocity_lock:
            self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': -0.3}}

    def sense_head_on(self):
        def process_sensor_data(msg):
            # check if an object is within a certain range
            ranges = msg['ranges']  # Assuming this is a LaserScan message
            front_range = ranges[len(ranges) // 2] 
            if front_range < 0.2286:  # half a tile
                self.object_detected = True
            else:
                self.object_detected = False
        
        # subscribe to the obstacle detection sensor data
        self.obstacle_sub.subscribe(process_sensor_data)

    def mow(self):
        # reset odometry
        reset_odom = roslibpy.Service(self.ros_node, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        reset_odom.call(roslibpy.ServiceRequest())

        # initialize for turns
        turn_counter = 1

        # subscribe to odometry to track position
        def odom_callback(msg):
            self.x_position = msg['pose']['pose']['position']['x']  # update the robot's x position

        self.odom_sub.subscribe(odom_callback)

        # start the sense thread for obstacle detection
        self.sense_head_on()

        # start the velocity thread to move the robot
        self.velocity_thread = th.Thread(target=self.move_robot, daemon=True)
        self.velocity_thread.start()

        # want the robot to stop once its distance is 4.572 meters to the right (10 tiles)
        target_distance = 4.572
        start_position = self.x_position

        while self.x_position - start_position < target_distance and not self.object_detected:
            self.drive_straight(2.286)  # drive straight 2.286 meters each time (~5 tiles)

            if self.object_detected:    # stop if object detected
                if turn_counter % 2 == 0:
                    self.turn_right()
                    self.drive_straight(0.2)  # drive forward a little after turning
                    self.turn_right()
                else:
                    self.turn_left()
                    self.drive_straight(0.2)  # drive forward a little after turning
                    self.turn_left()
                turn_counter += 1
                continue  # go back to driving straight and sensing in parallel

            time.sleep(0.1)

        # stop the sense and velocity threads when done
        self.stop()
        self.cleanup()

        idle = idleMode(self.ros_node, self.robot_name)
        idle.activate()
