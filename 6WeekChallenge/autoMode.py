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
        self.odom_sub.subscribe(self.odom_callback)

        # subscribe to sensor for obstacle detection
        self.ir_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')  

        # initialize for velocity control
        self.velocity_lock = th.Lock()
        self.running = False
        self.vel_thread = None

        # robot state
        self.x_position = 0.0  # initial position of the robot
        self.y_position = 0.0 
        self.orientation = 0.0  # orientation in radians (yaw)
        self.object_detected = False  # flag to track if an obstacle is detected
        
        # LED blinking event
        self.blinking_event = th.Event()
        self.blink_thread = None

        # PID controller parameters
        self.kp = 0.4  # Proportional gain
        self.ki = 0.12  # Integral gain
        self.kd = 0.5  # Derivative gain
        
        self.prev_error = 0.0 
        self.integral = 0.0 

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
            self.blink_thread.join()  # stop the blinking on the LED
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
        # calculate the target distance in terms of odometry
        target_distance = meters
        last_x = self.x_position
        last_y = self.y_position

        # set velocity to drive straight
        velocity_msg = {'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

        # start the velocity publishing thread
        self.running = True
        self.vel_thread = th.Thread(target=self.velocity_publisher, args=(velocity_msg,), daemon=True)
        self.vel_thread.start()

        # wait until the robot covers the desired distance
        while self.running:
            # calculate the change in distance since the last update
            delta_x = self.x_position - last_x
            delta_y = self.y_position - last_y
            current_distance = math.sqrt(delta_x**2 + delta_y**2)

            # check if we've reached or exceeded the target distance
            if current_distance >= target_distance:
                self.stop_robot()  # stop the robot as soon as the target distance is reached
                time.sleep(0.5)  # add a short wait to allow robot to stabilize
                self.running = False  # exit the loop once the robot stops
            
            time.sleep(0.1)  # delay to prevent blocking

    def stop_robot(self):
        # stop the robot by sending a zero velocity message
        stop_msg = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        if self.ros_node.is_connected:
            self.vel_pub.publish(roslibpy.Message(stop_msg))
        self.running = False  # Stop the velocity publishing thread
        print("Robot stopped.")

    def turn(self, target_angle):
        # Add the target angle to the current orientation
        target_angle = math.radians(target_angle)  # convert to radians
        target_yaw = self.orientation + target_angle  # new target is the current yaw + desired change

        self.turning = True
        print(f"Starting turn. Target yaw: {target_yaw}, Current yaw: {self.orientation}, Target change: {target_angle}")

        while self.turning:
            # calculate the error from the new target
            error = target_yaw - self.orientation

            # wrap to pi
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi

            # integral and derivative calculations
            self.integral += error
            derivative = error - self.prev_error

            # PID control signal
            control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative

            # update previous error
            self.prev_error = error

            # send the control signal as angular velocity
            turn_msg = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': control_signal}}
            if self.ros_node.is_connected:
                self.vel_pub.publish(roslibpy.Message(turn_msg))

            # check if reached the target angle within reasonable margin
            if abs(error) < 0.05: 
                self.turning = False
                time.sleep(0.5)  # brief wait to stabilize the robot

            print(f"Current angle: {self.orientation}, Target yaw: {target_yaw}, Error: {error}, Control signal: {control_signal}")

            time.sleep(0.1)  # small delay to avoid overly high frequency updates

    def velocity_publisher(self, velocity_msg):
        while self.running:
            if self.ros_node.is_connected:
                self.vel_pub.publish(roslibpy.Message(velocity_msg))
            time.sleep(0.1)  # send velocity every 100ms

    def quaternion_to_euler(self, x, y, z, w): # written by ChatGPT to help extract data
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1.0 else math.copysign(math.pi / 2, sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw  # roll, pitch, yaw in radians

    def odom_callback(self, msg):
        # extract position from the odometry message
        self.x_position = msg['pose']['pose']['position']['x']
        self.y_position = msg['pose']['pose']['position']['y']
        
        # extract orientation and convert to Euler angles
        quaternion = msg['pose']['pose']['orientation']
        roll, pitch, yaw = self.quaternion_to_euler(quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w'])
        
        # set the orientation
        self.orientation = yaw 

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
        reset_odom = roslibpy.Service(self.ros_node, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        reset_odom.call(roslibpy.ServiceRequest())

        final_pos = 2 # need to calculate in distance metric - 2 is a place holder
        self.sense_head_on() # need to run this as thread

        turn_counter = 1
        while self.x_position <= final_pos:
            self.drive_straight(1.5)
            if self.object_detected():
                if turn_counter % 2:
                    self.turn(-45)
                    self.drive_straight(0.1)
                    self.turn(-45)
                    turn_counter += 1
                else:
                    self.turn(45)
                    self.drive_straight(0.1)
                    self.turn(45)
                    turn_counter += 1

        if final_pos > self.x_position:
            self.cleanup()
            self.idle = idleMode(self.ros_node, self.robot_name)
            self.idle.activate()






        






# import pygame
# import time
# import threading as th
# import roslibpy
# import math
# from idleMode import idleMode

# class autoMode():
#     def __init__(self, ros_node, robot_name):
#         self.ros_node = ros_node
#         self.robot_name = robot_name
           
#         # create topics for publishing
#         self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
#         self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
#         self.vel_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/Twist')

#         # subscribe to odometry for position feedback
#         self.odom_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')

#         # subscribe to sensor for obstacle detection
#         self.ir_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')  

#         # initialize for velocity control
#         self.velocity_lock = th.Lock()
#         self.running = False
#         self.vel_thread = None

#         # robot state
#         self.x_position = 0.0  # initial  position of the robot
#         self.y_position = 0.0 
#         self.object_detected = False  # flag to track if an obstacle is detected
        
#         # LED blinking event
#         self.blinking_event = th.Event()
#         self.blink_thread = None

#     def activate(self):
#         # initialize joystick
#         pygame.init()
#         self.joystick = pygame.joystick.Joystick(0)
#         self.joystick.init()

#         # perform auto-specific actions
#         self.publish_flashing_yellow()
#         self.make_auto_noise()
#         self.mow()

#     def cleanup(self):
#         # prepare to exit mode
#         self.running = False
#         if self.vel_thread is not None:
#             self.vel_thread.join()  # stop the velocity thread
#         self.blinking_event.clear()  
#         if self.blink_thread and self.blink_thread.is_alive():
#             self.blink_thread.join() # stop the blinking on the LED
#         pygame.quit()
#         print("Exiting auto mode.")

#     def publish_flashing_yellow(self):
#         def blink_leds():
#             color = {'yellow': [{'red': 255, 'green': 255, 'blue': 0}] * 6}
#             while self.blinking_event.is_set():
#                 led_msg = {'leds': color['yellow'], 'override_system': True}
#                 if self.ros_node.is_connected:
#                     self.led_pub.publish(roslibpy.Message(led_msg))
#                 time.sleep(1)  # blink on
#                 led_msg = {'leds': color['yellow'], 'override_system': False}
#                 if self.ros_node.is_connected:
#                     self.led_pub.publish(roslibpy.Message(led_msg))
#                 time.sleep(1)  # blink off
        
#         # start the blinking thread
#         self.blinking_event.set()
#         self.blink_thread = th.Thread(target=blink_leds, daemon=True)
#         self.blink_thread.start()

#     def make_auto_noise(self):
#         # publish distinct auto noise
#         if self.ros_node.is_connected:
#             noise_msg = {
#                 'notes': [
#                     {'frequency': 680, 'max_runtime': {'sec': 0, 'nanosec': 100000000}},
#                     {'frequency': 370, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
#                     {'frequency': 680, 'max_runtime': {'sec': 0, 'nanosec': 100000000}},
#                     {'frequency': 370, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
#                 ]
#             }
#             self.noise_pub.publish(roslibpy.Message(noise_msg))

#     def drive_straight(self, meters):
#         start_position = [self.x_position, self.y_position]
#         target_position = start_position[1] + meters # only want to move in y

#         # set forward velocity
#         self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

#         # move the robot until it reaches the target position
#         while target_position - start_position[1] < meters and not self.object_detected:
#             self.vel_pub.publish(roslibpy.Message(self.current_velocity))  # move forward

#         # stop the robot
#         self.stop()

#     def stop(self):
#         # stop the robot's movement by setting the velocity to zero
#         self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
#         if self.ros_node.is_connected:
#             self.vel_pub.publish(roslibpy.Message(self.current_velocity))

#     def turn_left(self):
#         with self.velocity_lock:
#             self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.3}}

#     def turn_right(self):
#         with self.velocity_lock:
#             self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': -0.3}}

#     def sense_head_on(self):
#         def process_sensor_data(message):
#             for reading in message['readings']:
#                 frame_id = reading['header']['frame_id']
#                 value = reading['value']
#                 if frame_id == 'ir_intensity_front_center_left' and value > 600:
#                     self.object_detected = True
#                     self.stop()
#                     print("Obstacle detected at front center left, stopping.")
#                 elif frame_id == 'ir_intensity_front_center_right' and value > 600:
#                     self.object_detected = True
#                     self.stop()
#                     print("Obstacle detected at front center right, stopping.")

#         self.ir_sub.subscribe(process_sensor_data)

#     def mow(self):
#         self.drive_straight(2)
#         # reset odometry
#         reset_odom = roslibpy.Service(self.ros_node, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
#         reset_odom.call(roslibpy.ServiceRequest())

#         # intialize for turns
#         turn_counter = 1

#         # subscribe to odometry to track position
#         def odom_callback(msg):
#             self.x_position = msg['pose']['pose']['position']['x']  # update the robot's x position
#             self.y_position = msg['pose']['pose']['position']['y']  # update the robot's x position

#         self.odom_sub.subscribe(odom_callback)

#         # start the sense thread for obstacle detection
#         sense_thread = th.Thread(target=self.sense_head_on)
#         sense_thread.start()

#         # want the robot to stop once its distance is 4.572 meters to the right (10 tiles)
#         target_x_position = 4.572
#         start_position = self.x_position

#         while self.x_position - start_position < target_x_position and not self.object_detected:
#             self.drive_straight(2.286)  # drive straight 2.286 meters each time (~5 tiles)

#             if self.object_detected:    # stop if object detected
#                 if turn_counter % 2 == 0:
#                     self.turn_right()
#                     self.drive_straight(0.2)  # drive forward a little after turning
#                     self.turn_right()
#                 else:
#                     self.turn_left()
#                     self.drive_straight(0.2)  # drive forward a little after turning
#                     self.turn_left()
#                 turn_counter += 1
#                 continue  # go back to driving straight and sensing in parallel

#         time.sleep(1)

#         # stop the sense thread when done
#         self.running = False
#         #sense_thread.join()

#         # when the robot reaches the end, switch to idle mode
#         print("Mowing complete - switching to idle mode.")
#         self.cleanup()

#         idle = idleMode(self.ros_node, self.robot_name)
#         idle.activate()

            