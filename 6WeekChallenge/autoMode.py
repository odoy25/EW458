import pygame
import time
import threading as th
import roslibpy
import math
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
        self.ir_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')  

        # initialize for velocity control
        self.velocity_lock = th.Lock()
        self.running = False
        self.vel_thread = None

        # robot state
        self.x_position = 0.0  # initial  position of the robot
        self.y_position = 0.0 
        self.object_detected = False  # flag to track if an obstacle is detected
        
        # LED blinking event
        self.blinking_event = th.Event()
        self.blink_thread = None

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
            self.vel_thread.join()  # stop the velocity thread
        self.blinking_event.clear()  
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join() # stop the blinking on the LED
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

    def drive_straight(self, meters):
        start_position = [self.x_position, self.y_position]
        target_position = start_position[1] + meters # only want to move in y

        # set forward velocity
        self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

        # move the robot until it reaches the target position
        while target_position - start_position[1] < meters and not self.object_detected:
            self.vel_pub.publish(roslibpy.Message(self.current_velocity))  # move forward

        # stop the robot
        self.stop()

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
        def process_sensor_data(message):
            for reading in message['readings']:
                frame_id = reading['header']['frame_id']
                value = reading['value']
                if frame_id == 'ir_intensity_front_center_left' and value > 600:
                    self.object_detected = True
                    self.stop()
                    print("Obstacle detected at front center left, stopping.")
                elif frame_id == 'ir_intensity_front_center_right' and value > 600:
                    self.object_detected = True
                    self.stop()
                    print("Obstacle detected at front center right, stopping.")

        self.ir_sub.subscribe(process_sensor_data)

    def mow(self):
        self.drive_straight(2)
        # reset odometry
        reset_odom = roslibpy.Service(self.ros_node, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        reset_odom.call(roslibpy.ServiceRequest())

        # intialize for turns
        turn_counter = 1

        # subscribe to odometry to track position
        def odom_callback(msg):
            self.x_position = msg['pose']['pose']['position']['x']  # update the robot's x position
            self.y_position = msg['pose']['pose']['position']['y']  # update the robot's x position

        self.odom_sub.subscribe(odom_callback)

        # start the sense thread for obstacle detection
        sense_thread = th.Thread(target=self.sense_head_on)
        sense_thread.start()

        # want the robot to stop once its distance is 4.572 meters to the right (10 tiles)
        target_x_position = 4.572
        start_position = self.x_position

        while self.x_position - start_position < target_x_position and not self.object_detected:
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

        time.sleep(1)

        # stop the sense thread when done
        self.running = False
        #sense_thread.join()

        # when the robot reaches the end, switch to idle mode
        print("Mowing complete - switching to idle mode.")
        self.cleanup()

        idle = idleMode(self.ros_node, self.robot_name)
        idle.activate()

            