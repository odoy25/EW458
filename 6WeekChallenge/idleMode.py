import roslibpy

class idleMode:
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name

        # Create ROS topics for publishing LED and audio commands
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

        # Advertise topics
        self.led_pub.advertise()
        self.noise_pub.advertise()

    def publish_blue(self):
        """Publish a blue LED signal to the robot."""
        color = {'blue': [{'red': 0, 'green': 0, 'blue': 255}] * 6}
        if self.ros_node.is_connected:
            led_msg = {'leds': color['blue'], 'override_system': True}
            self.led_pub.publish(roslibpy.Message(led_msg))
        else:
            print("ROS node not connected. Cannot publish LED message.")

    def make_idle_noise(self):
        """Publish a three-chimed idle noise signal to the robot."""
        if self.ros_node.is_connected:
            noise_msg = {
                'notes': [
                    {'frequency': 440, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},  # First chime
                    {'frequency': 550, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},  # Second chime
                    {'frequency': 660, 'max_runtime': {'sec': 0, 'nanosec': 200000000}}   # Third chime
                ]
            }
            self.noise_pub.publish(roslibpy.Message(noise_msg))
        else:
            print("ROS node not connected. Cannot publish noise message.")

    def close(self):
        """Unadvertise and uninitialize topics before shutting down."""
        if self.led_pub.is_advertised:
            self.led_pub.unadvertise()
        if self.noise_pub.is_advertised:
            self.noise_pub.unadvertise()

