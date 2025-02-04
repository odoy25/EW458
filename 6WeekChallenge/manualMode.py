import pygame 
import time
import threading as th
import roslibpy

class manualMode():
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name

        # Create ROS topics for publishing LED and audio commands
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.vel_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')

        # Advertise topics
        self.led_pub.advertise()
        self.noise_pub.advertise()

    def publish_green(self):
        """Publish a green LED signal to the robot."""
        color = {'green': [{'red': 0, 'green': 255, 'blue': 0}] * 6}
        if self.ros_node.is_connected:
            led_msg = {'leds': color['green'], 'override_system': True}
            self.led_pub.publish(roslibpy.Message(led_msg))
        else:
            print("ROS node not connected. Cannot publish LED message.")

    def make_manual_noise(self):
        """Publish a two-chimed idle noise signal to the robot."""
        if self.ros_node.is_connected:
            noise_msg = {
                'notes': [
                    {'frequency': 320, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},  # First chime
                    {'frequency': 570, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},  # Second chime
                ]
            }
            self.noise_pub.publish(roslibpy.Message(noise_msg))
        else:
            print("ROS node not connected. Cannot publish noise message.")

    def steer(self):
        # Define movement commands
        directions = {
            "left": {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.6}},
            "right": {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': -0.6}},
            "straight": {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
            "stop": {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
        }

        self.velocity_lock = th.Lock()
        self.current_velocity = directions["stop"]
        self.running = True  # Thread control flag

        def velocity_publisher():
            while self.running:
                with self.velocity_lock:
                    if self.ros_node.is_connected:
                        self.vel_pub.publish(roslibpy.Message(self.current_velocity))
                time.sleep(0.1)

        # Start publishing thread
        self.vel_thread = th.Thread(target=velocity_publisher, daemon=True)
        self.vel_thread.start()

        # Initialize pygame for joystick input
        pygame.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick detected.")
            return

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        # Track axis states
        axis_values = {4: -1.0, 5: -1.0}  # Default trigger positions (-1.0 means unpressed)

        # Event loop for joystick control
        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        if event.axis in axis_values:
                            axis_values[event.axis] = event.value

                        left_trigger_active = axis_values[4] > -0.9
                        right_trigger_active = axis_values[5] > -0.9

                        # Update velocity safely
                        with self.velocity_lock:
                            if left_trigger_active and right_trigger_active:
                                self.current_velocity = directions["straight"]
                            elif left_trigger_active:
                                self.current_velocity = directions["left"]
                            elif right_trigger_active:
                                self.current_velocity = directions["right"]
                            else:
                                self.current_velocity = directions["stop"]

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            pygame.quit()
  

    def close(self):
        """Unadvertise and uninitialize topics before shutting down."""
        if self.led_pub.is_advertised:
            self.led_pub.unadvertise()
        if self.noise_pub.is_advertised:
            self.noise_pub.unadvertise()