import pygame   # use for reference: https://www.pygame.org/docs/ref/joystick.html 
import time
import roslibpy
from idleMode import idleMode
from manualMode import manualMode
from autoMode import autoMode

class robotController:
    def __init__(self):
        # establish connection as a ROS node
        self.ros_node = roslibpy.Ros(host='192.168.8.104', port=9012)
        self.ros_node.run()
        self.robot_name = 'bravo'

        # import modes from classes
        self.idle = idleMode(self.ros_node, self.robot_name)
        self.auto = autoMode(self.ros_node, self.robot_name)
        self.manual = manualMode(self.ros_node, self.robot_name)

    def run(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected! Exiting...")
            self.ros_node.terminate()
            pygame.quit()
            return

        joy = pygame.joystick.Joystick(0)
        joy.init()

        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 0:  # A button - MANUAL
                            self.manual.publish_green()
                            self.manual.make_manual_noise()
                            self.manual.steer()
                        elif event.button == 1:  # B button - NOTHING
                            pass
                        elif event.button == 2:  # X button - IDLE
                            self.idle.publish_blue()
                            self.idle.make_idle_noise()  
                        elif event.button == 3:  # Y button - AUTO
                            self.auto.publish_flashing_yellow()
                            self.auto.make_auto_noise()
                            self.auto.mow()
                        

        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.ros_node.terminate()
            pygame.quit()


if __name__ == '__main__':
    controller = robotController()
    controller.run()

        

