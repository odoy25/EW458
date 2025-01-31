import pygame   # use for reference: https://www.pygame.org/docs/ref/joystick.html 
import time
import threading as th
import roslibpy

ros_node = roslibpy.Ros(host='192.168.8.104', port=9012)
ros_node.run()

robot_name = 'bravo'

led_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
vel_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')

colors = {
    'red': [{'red': 255, 'green': 0, 'blue': 0}] * 6,
    'green': [{'red': 0, 'green': 255, 'blue': 0}] * 6,
    'blue': [{'red': 0, 'green': 0, 'blue': 255}] * 6,
    'yellow': [{'red': 255, 'green': 255, 'blue': 0}] * 6
}

left = {
    'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.6}
}

right = {
    'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': -0.6}
}

straight = {
    'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

stop = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

blinking_event = th.Event()
velocity_lock = th.Lock()
current_velocity = stop
running = True  # for velocity publishing thread

def blink_leds(color):
    while blinking_event.is_set(): 
        led_msg = {'leds': colors[color], 'override_system': True}
        if ros_node.is_connected:
            led_pub.publish(roslibpy.Message(led_msg))
        time.sleep(1)
        led_msg = {'leds': colors[color], 'override_system': False}
        if ros_node.is_connected:
            led_pub.publish(roslibpy.Message(led_msg))
        time.sleep(1)

def velocity_publisher():
    global current_velocity # used chatGPT for this
    while running:
        with velocity_lock:
            if ros_node.is_connected:
                vel_pub.publish(roslibpy.Message(current_velocity))
        time.sleep(0.1)

vel_thread = th.Thread(target=velocity_publisher, daemon=True)
vel_thread.start()

def main():
    global blinking_event, current_velocity #chatGPT
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f'number of buttons: {joy.get_numbuttons()}, number of hats: {joy.get_numhats()}, number of axes: {joy.get_numaxes()}')

    arm_count = 0  # to track arming/disarming
    color = 'yellow'  # default color
    blink_thread = None  # Reference to the blinking thread
    left_trigger_active = False
    right_trigger_active = False
    dead_zone = 0.1  # Dead zone threshold

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # A button
                        color = 'green'
                    elif event.button == 1:  # B button
                        color = 'red'
                    elif event.button == 2:  # X button
                        color = 'blue'
                    elif event.button == 3:  # Y button
                        color = 'yellow'

                    led_msg = {'leds': colors[color], 'override_system': True} #chatGPT made these three lines to cut down on`redundancy`
                    if ros_node.is_connected:
                        led_pub.publish(roslibpy.Message(led_msg))

                    if event.button == 7:  # Menu button (Arming/Disarming)
                        arm_count += 1  
                        if arm_count % 2 == 0:  # Disarmed - SOLID
                            print('Disarmed')
                            blinking_event.clear()
                            if blink_thread and blink_thread.is_alive():
                                blink_thread.join()  # stop the previous blink thread before starting a new one
                            led_msg = {'leds': colors[color], 'override_system': True}
                            if ros_node.is_connected:
                                led_pub.publish(roslibpy.Message(led_msg))
                        else:  # Armed - BLINKING
                            print('Armed')
                            blinking_event.set()
                            if blink_thread and blink_thread.is_alive():
                                blink_thread.join()  # stop any existing blinking thread is stopped
                            blink_thread = th.Thread(target=blink_leds, args=(color,), daemon=True)
                            blink_thread.start()

                if event.type == pygame.JOYHATMOTION:
                    if event.hat == 0:
                        if event.value == (0, 1):  # Up
                            print('Going manual...')
                        elif event.value == (0, -1):  # Down
                            print('Going automatic...')
                        elif event.value == (1, 0):  # Right
                            print('right')
                        elif event.value == (-1, 0):  # Left
                            print('left')

                if event.type == pygame.JOYAXISMOTION:
                    if event.axis == 4:  # Left trigger
                        left_trigger_active = event.value > -0.9 # dead zone
                    elif event.axis == 5:  # Right trigger
                        right_trigger_active = event.value > -0.9

                # update velocity with lock 
                with velocity_lock:
                    if left_trigger_active and right_trigger_active:
                        current_velocity = straight
                    elif left_trigger_active:
                        current_velocity = left
                    elif right_trigger_active:
                        current_velocity = right
                    else:
                        current_velocity = stop

    except KeyboardInterrupt:
        print("Shutting down...")

        # stop velocity 
        global running
        running = False
        vel_thread.join()

        # stop the robot 
        with velocity_lock:
            current_velocity = stop
            if ros_node.is_connected:
                vel_pub.publish(roslibpy.Message(stop))

        # stop everything
        ros_node.terminate()
        pygame.quit()
        return

if __name__ == '__main__':
    main()
