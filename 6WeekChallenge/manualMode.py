import pygame
import time
import threading as th
import roslibpy

class manualMode:
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name

        # create topics for publishing
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.vel_pub = roslibpy.Topic(self.ros_node, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')

        # initialize pygame for joystick input
        self.joystick = None
        self.velocity_lock = th.Lock()
        self.running = False
        self.vel_thread = None

    def activate(self):
        # initialize joystick
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # start the velocity publishing thread
        self.running = True
        self.vel_thread = th.Thread(target=self.velocity_publisher, daemon=True)
        self.vel_thread.start()

        # perform manual-specific actions
        self.publish_green()
        self.make_manual_noise()

    def cleanup(self):
        # prepare to exit mode
        self.running = False 
        if self.vel_thread is not None:
            self.vel_thread.join() # stop the velocity thread
        pygame.quit()
        print("Exiting manual mode.")

    def velocity_publisher(self):
        while self.running:
            self.update_velocity()
            if self.ros_node.is_connected:
                self.vel_pub.publish(roslibpy.Message(self.current_velocity))
            time.sleep(0.05)  # publish every 50ms

    def publish_green(self):
        # publish distinct manual color (green)
        color = {'green': [{'red': 0, 'green': 255, 'blue': 0}] * 6}
        if self.ros_node.is_connected:
            led_msg = {'leds': color['green'], 'override_system': True}
            self.led_pub.publish(roslibpy.Message(led_msg))

    def make_manual_noise(self):
        # publish distinct manual noise
        if self.ros_node.is_connected:
            noise_msg = {
                'notes': [
                    {'frequency': 320, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
                    {'frequency': 570, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
                ]
            }
            self.noise_pub.publish(roslibpy.Message(noise_msg))

    def update_velocity(self):
        # read joystick for manual movement
        self.axis_values = {
            4: self.joystick.get_axis(4),  # Left trigger
            5: self.joystick.get_axis(5)   # Right trigger
        }

        # apply deadzone and update velocity
        DEADZONE = 0.1
        left_trigger_active = self.axis_values[4] > DEADZONE
        right_trigger_active = self.axis_values[5] > DEADZONE

        with self.velocity_lock:
            if left_trigger_active and right_trigger_active:
                self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
            elif left_trigger_active:
                self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.6}}
            elif right_trigger_active:
                self.current_velocity = {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': -0.6}}
            else:
                self.current_velocity = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}