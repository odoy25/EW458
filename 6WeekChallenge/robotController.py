import pygame  # use for reference: https://www.pygame.org/docs/ref/joystick.html 
import roslibpy
from idleMode import idleMode
from manualMode import manualMode
from autoMode import autoMode 

class robotController:
    def __init__(self):
        # establish connection as a ROS node
        self.ros_node = roslibpy.Ros(host='127.0.0.1', port=9012) # roslibpy.Ros(host='192.168.8.104', port=9012)
        self.ros_node.run()
        self.robot_name = 'juliet'

        # initialize modes from classes
        self.idle = idleMode(self.ros_node, self.robot_name)
        self.auto = autoMode(self.ros_node, self.robot_name)
        self.manual = manualMode(self.ros_node, self.robot_name)

        # set initial mode to idle
        self.current_mode = self.idle

    def switch_mode(self, mode):
        # allow for ease when switching between modes (cleanup and activate are within each class, used chatGPT for help in development because we had trouble switching between modes intially)
        if self.current_mode != mode:
            self.current_mode.cleanup()
            self.current_mode = mode
            self.current_mode.activate()

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
                            self.switch_mode(self.manual)
                        elif event.button == 2:  # X button - IDLE
                            self.switch_mode(self.idle)
                        elif event.button == 3:  # Y button - AUTO
                            self.switch_mode(self.auto)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.ros_node.terminate()
            pygame.quit()

if __name__ == '__main__':
    controller = robotController()
    controller.run()


