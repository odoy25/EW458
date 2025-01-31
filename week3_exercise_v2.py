import pygame   # use for reference: https://www.pygame.org/docs/ref/joystick.html 
import time
import threading as th
import roslibpy

ros_node = roslibpy.Ros(host='192.168.8.104', port=9012)
ros_node.run()

robot_name = 'omega'

led_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
vel_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')

colors = {
    'red': [{'red': 255, 'green': 0, 'blue': 0}] * 6,
    'green': [{'red': 0, 'green': 255, 'blue': 0}] * 6,
    'blue': [{'red': 0, 'green': 0, 'blue': 255}] * 6,
    'yellow': [{'red': 255, 'green': 255, 'blue': 0}] * 6
}

def skid_steer(left_speed, right_speed):
    linear_x = (left_speed + right_speed) / 2.0
    angular_z = (right_speed - left_speed) / 2.0
    return {
        'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
    }

# Use threading.Event for thread-safe blinking control
blinking_event = th.Event()

def blink_leds(color):
    while blinking_event.is_set():  # Check if the event is set
        led_msg = {'leds': colors[color], 'override_system': True}
        led_pub.publish(roslibpy.Message(led_msg))
        time.sleep(1)
        led_msg = {'leds': colors[color], 'override_system': False}
        led_pub.publish(roslibpy.Message(led_msg))
        time.sleep(1)

def main():
    global blinking_event
    pygame.init()
    pygame.joystick.init()

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f'number of buttons: {joy.get_numbuttons()}, number of hats: {joy.get_numhats()}, number of axes: {joy.get_numaxes()}')

    arm_count = 0  # to track arming/disarming
    color = 'yellow'  # default color
    blink_thread = None  # Reference to the blinking thread

    last_publish_time = time.time()  # Track the time of the last velocity message

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # A button
                        color = 'green'
                        led_msg = {'leds': colors[color], 'override_system': True}
                        led_pub.publish(roslibpy.Message(led_msg))
                    elif event.button == 1:  # B button
                        color = 'red'
                        led_msg = {'leds': colors[color], 'override_system': True}
                        led_pub.publish(roslibpy.Message(led_msg))
                    elif event.button == 2:  # X button
                        color = 'blue'
                        led_msg = {'leds': colors[color], 'override_system': True}
                        led_pub.publish(roslibpy.Message(led_msg))
                    elif event.button == 3:  # Y button
                        color = 'yellow'
                        led_msg = {'leds': colors[color], 'override_system': True}
                        led_pub.publish(roslibpy.Message(led_msg))
                    elif event.button == 7:  # Menu button
                        arm_count += 1  
                        if arm_count % 2 == 0:  # Disarmed - SOLID
                            print('Disarmed')
                            blinking_event.clear()  # Stop blinking
                            if blink_thread and blink_thread.is_alive():
                                blink_thread.join()  # Ensure thread is stopped
                            led_msg = {'leds': colors[color], 'override_system': True}
                            led_pub.publish(roslibpy.Message(led_msg))
                        else:  # Armed - BLINKING
                            print('Armed')
                            blinking_event.set()  # Start blinking
                            blink_thread = th.Thread(target=blink_leds, args=(color,), daemon=True)
                            blink_thread.start()
                if event.type == pygame.JOYAXISMOTION:
                    left_speed = -joy.get_axis(1)  # Left stick Y-axis
                    right_speed = -joy.get_axis(3)  # Right stick Y-axis

                    # Publish velocity message at 10 Hz
                    current_time = time.time()
                    if current_time - last_publish_time >= 0.1:  # 0.1 seconds = 10 Hz
                        vel_pub.publish(roslibpy.Message(skid_steer(left_speed, right_speed)))
                        last_publish_time = current_time  # Update the last publish time
    except KeyboardInterrupt:
        # Turn off LED
        blinking_event.clear()  # Ensure blinking thread stops
        if blink_thread and blink_thread.is_alive():
            blink_thread.join()
        led_msg = {'leds': colors[color], 'override_system': False}
        led_pub.publish(roslibpy.Message(led_msg))
        led_pub.unadvertise()
        ros_node.terminate()

        print('Exiting.')
        pygame.quit()
        return

if __name__ == '__main__':
    main()
