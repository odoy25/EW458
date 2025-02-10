import roslibpy
import pygame

class idleMode:
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name

        # create topics for publishing
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

    def activate(self):
        # initialize joystick
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # perform idle-specific actions
        self.publish_blue()
        self.make_idle_noise()

    def cleanup(self):
        # prepare to exit mode
        pygame.quit()
        print("Exiting idle mode.")

    def publish_blue(self):
        # publish distinct idle color (blue)
        color = {'blue': [{'red': 0, 'green': 0, 'blue': 255}] * 6}
        if self.ros_node.is_connected:
            led_msg = {'leds': color['blue'], 'override_system': True}
            self.led_pub.publish(roslibpy.Message(led_msg))

    def make_idle_noise(self):
        # publish distinct idle noise
        if self.ros_node.is_connected:
            noise_msg = {
                'notes': [
                    {'frequency': 440, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
                    {'frequency': 550, 'max_runtime': {'sec': 0, 'nanosec': 200000000}},
                    {'frequency': 660, 'max_runtime': {'sec': 0, 'nanosec': 200000000}}
                ]
            }
            self.noise_pub.publish(roslibpy.Message(noise_msg))