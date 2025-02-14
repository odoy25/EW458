import pygame
from pygame.locals import *
from math import cos, sin, degrees, radians, pi
import json
from autobahn.twisted.websocket import WebSocketServerFactory, WebSocketServerProtocol
from twisted.internet import reactor, task
import random
import numpy
import threading
import time
import os

class Create3(pygame.sprite.Sprite):
    """
This module simulates a Create3 robot using Pygame and integrates with ROS via WebSocket.
Classes:
    Create3(pygame.sprite.Sprite): Represents the Create3 robot with methods to update its state, check for collisions, and publish odometry.
    Topic: Represents a ROS topic with methods to publish and subscribe to messages.
    WebSocketProtocol(WebSocketServerProtocol): Handles WebSocket connections and message broadcasting for ROS topics.
    RosSimulator: Manages the Pygame simulation environment, including robot creation, background setup, and the main simulation loop.
Functions:
    main(): Initializes the ROS simulator, sets up the WebSocket server, and starts the Pygame simulation loop integrated with the Twisted reactor.
Usage:
    Run this module directly to start the Create3 robot simulation with ROS integration.
"""
    def __init__(self, screen, ros_instance, name='juliet'):
        super().__init__()
        
        print(f'Creating robot {name}')
        # check if ./create3.png exists
        load_image = os.path.exists('./create3.png')
        load_image = 1

        if load_image:
            self.image = pygame.image.load('./create3.png').convert_alpha()
            self.image = pygame.transform.rotate(self.image, -90)
            self.image = pygame.transform.smoothscale(self.image, (self.image.get_width() // 5, self.image.get_height() // 5))
        else:
            # make a circle
            r = 30
            self.image = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
            pygame.draw.circle(self.image, (0,0,0), (r, r), r)
            # draw a smaller circle for the center
            pygame.draw.circle(self.image, (150,150,150), (r, r), r-5)

        self.radius = self.image.get_width() // 2 -10# radius of robot in pixels
        self.radius_points = []
        for i in range(0, 360, 30):
            self.radius_points.append((self.radius * cos(i), self.radius * sin(i)))
        self.ir_points = [-65.3,-34,-14.25,3,20,38,65.3] # in degrees, from create3 technical specs
        self.IR_RANGE = 0.1 # in meters

        self.og_image = self.image
        self.rect = self.image.get_rect(center=screen.get_rect().center)
        self.robot_width = self.rect.width
        self.screen = screen

        # Robot state
        self.pixel_per_meter = 250
        self.theta = 0 # in radians
        self.theta_dot = 0 # in radians per second
        self.x, self.y = (0,0)
        self.rect.center = self.get_pixel_position()
        self.v = 0                 # linear velocity in m/s
        self.ir_measurements = [0]*len(self.ir_points)

        
        # timing variables
        self.fps = 60
        self.dt = 1 / self.fps

        # led lights (list of 6 red, green, blue values as dict keys)
        self.light_vector = []
        #self.light_vector = [{'red':random.randint(0,255), 'green':random.randint(0,255), 'blue':random.randint(0,255)} for i in range(6)]
        self.draw_light_ring()

        # ROS topics
        self.ros = ros_instance
        self.cmd_vel_topic = Topic(ros_instance, f'/{name}/cmd_vel', 'geometry_msgs/Twist')
        self.odom_topic = Topic(ros_instance, f'/{name}/odom', 'nav_msgs/Odometry')
        self.imu_topic = Topic(ros_instance, f'/{name}/imu', 'sensor_msgs/Imu')
        self.ir_topic = Topic(ros_instance, f'/{name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
        self.light_topic = Topic(ros_instance, f'/{name}/cmd_lightring', 'irobot_create_msgs/LightVector')
        self.audio_topic = Topic(ros_instance, f'/{name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.audio_topic.msg = None

        # callback to set lights
        self.light_topic.subscribe(self.set_lights)

    def update(self):
        # Update velocities from cmd_vel topic

        if self.cmd_vel_topic.msg and not self.cmd_vel_topic.has_timed_out():
            # we have a message and it is not timed out
            self.v = self.cmd_vel_topic.msg['linear']['x']
            self.theta_dot = self.cmd_vel_topic.msg['angular']['z']
        
        else:
            # no message or timed out
            self.v = 0
            self.theta_dot = 0

        # Calculate new position in METERS with TIME_STEP
        new_x = self.x + self.v * cos(self.theta) * self.dt 
        new_y = self.y + self.v * sin(self.theta) * self.dt
        self.theta = self.theta + self.theta_dot * self.dt # robot always rotates

        if not self.check_collision(new_x, new_y):
            # no collision!
            self.collision = False
            self.x, self.y = new_x, new_y
        else:
            self.collision = True

        # generate IR measurements
        self.ir_measurements = self.measure_IR(self.x, self.y)
        self.rect.center = self.get_pixel_position()
        self.image = self.og_image

        # play audio if message comes through
        if self.audio_topic.msg is not None:
            threading.Thread(target=self.play_audio, args=(self.audio_topic.msg['notes'][0]['note'],self.audio_topic.msg['notes'][0]['duration']), daemon=True).start()
            self.audio_topic.msg = None
        
        # Blit the light ring that has already been created
        self.screen.blit(self.light_ring, self.light_ring_rect)
        self.image = pygame.transform.rotate(self.image, degrees(self.theta))
        self.rect = self.image.get_rect(center=self.rect.center)

        # Publish sensor messages
        self.publish_odom()
        self.publish_imu()
        self.publish_ir()

    def draw_light_ring(self):
        # make a new surface to draw the light ring
        ring_radius = 40
        ring_radius_extended = 50
        ring_width = 10
        self.light_ring = pygame.Surface((2*ring_radius, 2*ring_radius), pygame.SRCALPHA)
        self.light_ring.fill((0,0,0,0))
        self.light_ring_rect = self.light_ring.get_rect()
        cx, cy = self.light_ring_rect.center



        # loop over each pie slice and draw the color in a polygon
        for i, color in enumerate(self.light_vector):
            start_angle = radians(i * 60)
            end_angle = radians((i+1) * 60)
            pt1 = (cx, cy) # center of the circle
            pt2 = (cx + ring_radius_extended * cos(start_angle), cy + ring_radius_extended * sin(start_angle))
            pt3 = (cx + ring_radius_extended * cos(end_angle), cy + ring_radius_extended * sin(end_angle))
            pygame.draw.polygon(self.light_ring, (color['red'], color['green'], color['blue']), [pt1, pt2, pt3])

        # now cut a hole in the middle
        pygame.draw.circle(self.light_ring, (0,0,0,0), (cx, cy), ring_radius - ring_width)

        # now make a circl mask to make the light ring circular
        mask = pygame.Surface((2*ring_radius, 2*ring_radius), pygame.SRCALPHA)
        mask.fill((255,255,255,0))
        pygame.draw.circle(mask, (0,0,0,255), (cx, cy), ring_radius)
        self.light_ring.blit(mask, (0,0), special_flags=pygame.BLEND_RGB_ADD)


        # blit the light ring to the screen
        self.light_ring_rect.centerx = 50
        self.light_ring_rect.centery = 100

    def set_lights(self,msg):
        '''
        "{override_system: true, leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}]}"
        '''
        led_msg = msg.get('leds', None)
        # get the led msg
        if not led_msg:
            # no message, LIGHTS OFF
            self.light_vector = []
            return
        else:
            # set the light vector
            self.light_vector = led_msg
        
        # draw the light ring
        self.draw_light_ring()
  
    def check_collision(self,x_m, y_m):
        x, y = self.get_pixel_position(x_m, y_m) # convert meters to pixels
        
        # look at pixels around the robot radius and check if any of them are walls
        for point in self.radius_points:
            if self.check_wall((x + point[0], y + point[1])):
                return True
        return False

    def get_pixel_position(self, x = None, y = None):
        # if no x, y given, use current self.x, self.y
        if None in (x, y):
            x, y = self.x, self.y
        # take an x,y position in meters and set the pixel position
        pixel_center = self.screen.get_rect().center
        return (x * self.pixel_per_meter + pixel_center[0], pixel_center[1] - y * self.pixel_per_meter)

    def check_wall(self, point):
        #checks pixel value at point and returns True if it is a wall
        point = (int(point[0]), int(point[1]))
        rgb_val = self.ros.background.get_at(point)
        return rgb_val[0] < 60  # Check for a "wall"

    def measure_IR(self,x_m,y_m):
        """Simulates LiDAR and returns range measurements."""
        # cx, cy is the FRONT of the robot
        px, py = self.get_pixel_position() # current position in pixels

        

        ranges = [] # range in pixels

        max_pixel_range = self.IR_RANGE * self.pixel_per_meter

        for angle in self.ir_points:
            
            # get front of robot in pixels
            fpx = self.radius * cos(self.theta+radians(angle)) + px
            fpy = self.radius * sin(-self.theta-radians(angle)) + py

            ray_dx = cos(self.theta + radians(angle))
            ray_dy = sin(-self.theta - radians(angle))
            distance = 0

            while distance < max_pixel_range:
                distance += 1 # in pixels (resolution distance)
                x = int(fpx + ray_dx * distance) # pixel x
                y = int(fpy + ray_dy * distance) # pixel y

                # Check if the ray collides with a wall or obstacle
                if self.check_wall((x,y)):
                    break
    
            # Store the distance ( for that angle)
            ranges.append(distance)
            # Draw the ray
            endpoint = (fpx + ray_dx * distance, fpy + ray_dy * distance)
            pygame.draw.line(self.screen, self.ros.colors.get('red'), (fpx, fpy), endpoint, 1)

            # TODO: add scaling to range to represent real IR numbers
        return [ r/max_pixel_range * 255 for r in ranges]

    def publish_odom(self):
        msg = {
                'pose': {
                    'position': {'x': self.x, 'y': self.y, 'z': 0},
                    'orientation': {'x': 0, 'y': 0, 'z': sin(self.theta/2), 'w': cos(self.theta/2)}
                },
                'twist': {
                    'linear': {'x': self.v, 'y': 0, 'z': 0},
                    'angular': {'x': 0, 'y': 0, 'z': self.theta_dot}
                }
            }
        
        self.publish_message('odom', msg)
    
    def publish_imu(self):
        # https://iroboteducation.github.io/create3_docs/api/odometry/
        msg = {
                'orientation': {'x': 0, 'y': 0, 'z': sin(self.theta/2), 'w': cos(self.theta/2)
                },
                'angular_velocity': {'x': 0, 'y': 0, 'z': self.theta_dot
                },
                'linear_acceleration': {'x': 0, 'y': 0, 'z': 0
                },
            }
        self.publish_message('imu', msg)

    def publish_ir(self):
        frame_names = [
            'side_left', 'left', 'front_left', 'front_center_left', 'front_center_right', 'front_right', 'right'
        ]
        frame_names.reverse()
        readings = [ {'header': {'frame_id': f'ir_intensity_{frame_names[i]}' }, 'value': self.ir_measurements[i]} for i in range(len(self.ir_measurements))]

        msg = { 'readings': readings }

        self.publish_message('ir_intensity', msg)
        
    def publish_message(self, topic_name, message):
        if self.ros.broadcast_payload:
            self.ros.broadcast_payload({
                    'op': 'publish',
                    'topic': f'/{self.ros.robot_name}/{topic_name}',
                    'msg': message
                })
    
    def play_audio(self, frequency, duration):
        print("here")
        sample_rate = 44100
        n_samples = int(round(duration*sample_rate))

        frequency_l = frequency
        frequency_r = frequency + 110

        #setup our numpy array to handle 16 bit ints, which is what we set our mixer to expect with "bits" up above
        buf = numpy.zeros((n_samples, 2), dtype = numpy.int16)
        max_sample = 2**(16 - 1) - 1

        for s in range(n_samples):
            t = float(s)/sample_rate    # time in seconds

            #grab the x-coordinate of the sine wave at a given time, while constraining the sample to what our mixer is set to with "bits"
            buf[s][0] = int(round(max_sample*sin(2*pi*frequency_l*t)))        # left
            buf[s][1] = int(round(max_sample*0.5*sin(2*pi*frequency_r*t)))    # right

        sound = pygame.sndarray.make_sound(buf)
        #play once, zero repeats
        sound.play(loops = 0)

class Topic:
    def __new__(cls, ros_instance, topic_name, message_type=''):
        # check if the topic already exists and return it!
        if topic_name in ros_instance.topic_dict:
            print('returning existing topic')
            return ros_instance.topic_dict[topic_name]
        else:
            # otherwise create a new topic
            return super(Topic, cls).__new__(cls)


    def __init__(self, ros_instance, topic_name, message_type='', timeout=1):
        self.ros = ros_instance
        self.topic_name = topic_name
        self.message_type = message_type
        # add the topic to the ros instance
        self.ros.add_topic(self)
        self.callbacks = []
        self.msg = None
        self.timeout = 1000 * timeout # in milliseconds
        self.clock = pygame.time.Clock()
        self.clock.tick()
        self.max_message_rate = 20 # in Hz
        self.avg_message_rate = 0
        self.msg_count = 0

    def publish(self, message):
        self.msg = message
        self.msg_count += 1

        # update the time
        self.clock.tick()
        # run all callbacks
        [callback(message) for callback in self.callbacks]

        # check to see if the message rate is too high
        delta_t = self.clock.get_time()
        if self.msg_count > 10:
            msg_rate = 1000 / delta_t
        else :
            msg_rate = 0
            self.first_message = False

        self.avg_message_rate = 0.9 * self.avg_message_rate + 0.1* msg_rate

        print(f"Message rate for {self.topic_name}: {self.avg_message_rate:.2f} Hz")

        if self.avg_message_rate > self.max_message_rate:
            print(f"Message rate too high for {self.topic_name}: {self.avg_message_rate:.2f} Hz")
            #self.ros.set_alert(f"Message rate too high for {self.topic_name}: {self.avg_message_rate:.2f} Hz")

    def has_timed_out(self):
        # returns True if the topic has timed out
        return self.clock.get_time() > self.timeout

    def subscribe(self, callback):
        # add the callback to the topic
        self.callbacks.append(callback)

class WebSocketProtocol(WebSocketServerProtocol):
    def __init__(self, ros_instance):
        super().__init__()
        self.ros = ros_instance

        self.robot_name = ros_instance.robot_name

    # if we get a connection, set the ros instance to connected
    def onConnect(self, request):
        self.ros.is_connected = True
        # clear alert message if connected
        self.ros.set_alert('')
        print(f'Connected to {request.peer}')

    def broadcast_message(self, payload):
        try:
            self.sendMessage(json.dumps(payload).encode('utf8'))
        except:
            self.ros.is_connected = False
            print('No Websocket Connection!' , end='\r') 
            self.ros.set_alert('Not Connected')    

    def onMessage(self, payload, isBinary):
        if not isBinary:
            try:
                message = json.loads(payload.decode('utf8'))
                # get the topic
                t_name = message.get('topic', '')
                if t_name in self.ros.topic_dict:
                    t = self.ros.topic_dict[t_name]
                    t.publish(message.get('msg', None))
            except:
                pass

class RosSimulator:
    def __init__(self, robot_name, host = None, port = None):
        pygame.mixer.pre_init(44100, -16, 2)
        pygame.init()
        self.version = '1.00'
        self.alert_msg = 'Not Connected'
        self.screen = pygame.display.set_mode((1000, 1000))
        self.clock = pygame.time.Clock()
        self.running = True
        self.robots = pygame.sprite.Group()
        self.is_connected = False
        self.topic_dict ={}  # dictionary with topic name as key and topic object as value
        self.colors = {
            # https://coolors.co/292f36-d7263d-ffffff-197bbd-058c42
            'grey': (41, 47, 54), # grey
            'red': (215, 38, 61), # red
            'white' : (255, 255, 255), # white
            'green' : (5,140,66), # green
            'blue' : (25,123,189) # blue
        }
        self.background = self.create_background()
        # set caption
        pygame.display.set_caption('Create3 Robot Simulation')

        self.is_connected = True
        # main player robot
        self.robot_name = robot_name
        self.main_robot = Create3(self.screen, self, robot_name)
        self.robots.add(self.main_robot)

        self.broadcast_payload = None # callback to send messages to the network

    def add_topic(self, topic):
        self.topic_dict[topic.topic_name] = topic
     

    def create_background(self):
        W = self.screen.get_width()
        H = self.screen.get_height()
        offset = 300
        wall_width = 40
        floor_color = self.colors.get('white', (255, 255, 255))
        wall_color = self.colors.get('blue', (0, 0 , 0))
        # returns background that is size of screen
        background = pygame.Surface(self.screen.get_size())
        background.fill(floor_color)
        
        # now make inside offset grey
        wall_rect = background.get_rect().inflate(-offset, -offset)
        background.fill(wall_color, wall_rect)

        yard_rect = wall_rect.inflate(-wall_width, -wall_width) 
        background.fill(floor_color, yard_rect)

        # put an opening on top/bottom wall
        door_width = 200
        doors_rect = pygame.rect.Rect(0,0, door_width, H)
        doors_rect.center = background.get_rect().center
        background.fill(floor_color, doors_rect)


        return background
    
    def set_alert(self, msg):
        self.alert_msg = msg

    def run_once(self):
        # Non-blocking loop to run game continuously!
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print('Exiting...')
                self.running = False

        # Draw background
        self.screen.blit(self.background, (0, 0))

        # Update all robots
        self.robots.update()
        self.robots.draw(self.screen)

        # put fps in top left corner
        fps = self.clock.get_fps()
        font = pygame.font.Font(None, 24)
        text = font.render(f'v{self.version} FPS: {fps:.2f}', True, self.colors.get('green', (0, 0, 255)))
        self.screen.blit(text, (10, 10))

        # if alert message, put it below fps
        if self.alert_msg:
            text = font.render(self.alert_msg, True, self.colors.get('red', (255, 0, 0)))
            self.screen.blit(text, (10, 40))


        # if 'p' pressed, take a screenshot
        keys = pygame.key.get_pressed()
        if keys[pygame.K_p]:
            pygame.image.save(self.screen, f'assets/screenshots/{random.randint(0, 1000)}.png')
            # sleep for 1 second to prevent multiple screenshots
            pygame.time.wait(1000)

        
        self.clock.tick(60)
        pygame.display.flip()
    
    def run_with_exit(self, reactor):
        if self.running:
            self.run_once()
        else: 
            pygame.quit()
            reactor.stop()

def main():

    ip = '0.0.0.0'
    port = 9012
    robot_name = 'juliet'


    ros = RosSimulator(robot_name)

    # Set up WebSocket server
    factory = WebSocketServerFactory(f"ws://{ip}:{port}")
    factory.protocol = lambda: WebSocketProtocol(ros)
    reactor.listenTCP(port, factory)

    # Integrate Pygame loop with Twisted reactor
    pygame_task = task.LoopingCall(ros.run_with_exit, reactor)
    pygame_task.start(1/61)

    print(f"Simulated Robot: {robot_name} on ws://{ip}:{port}")
    reactor.run()

if __name__ == "__main__":
    main()
