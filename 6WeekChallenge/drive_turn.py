import roslibpy
import threading as th
import time
import math

class robotController():
    def __init__(self):
        self.ros_node = roslibpy.Ros(host='192.168.8.104', port=9012)
        self.ros_node.run()
        self.robot_name = 'echo'
           
        # Create topics for publishing
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.noise_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.vel_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/Twist')

        self.odom_sub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')
        self.odom_sub.subscribe(self.odom_callback)

        # Initialize for velocity control
        self.running = False
        self.current_position = None  # Track odometry data
        self.current_yaw = None  # Track current orientation

    def odom_callback(self, message):
        """Extracts the current robot position and orientation from odometry."""
        if 'pose' in message and 'pose' in message['pose']:
            self.current_position = message['pose']['pose']['position']
            orientation = message['pose']['pose']['orientation']
            
            # Convert quaternion to Euler angles
            self.current_yaw = self.quaternion_to_euler(orientation)
            print(f"Odometry Update: x={self.current_position['x']:.3f}, y={self.current_position['y']:.3f}, yaw={math.degrees(self.current_yaw):.1f}°")

    def quaternion_to_euler(self, quaternion):
        """Converts a quaternion to yaw (Z rotation)."""
        x, y, z, w = quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']
        
        # Yaw (Z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw  # Radians

    def normalize_angle(self, angle):
        """Normalize the angle to be within the range -π to π."""
        angle = math.fmod(angle + math.pi, 2 * math.pi)
        if angle < 0:
            angle += 2 * math.pi
        return angle - math.pi

    def shortest_angle_difference(self, current_angle, target_angle):
        """Calculates the shortest angle difference between two angles."""
        diff = target_angle - current_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def drive_straight(self, speed=0.2, distance=2):
        """Drives the robot straight at a given speed for a specified distance using odometry."""
        while self.current_position is None:
            print("Waiting for odometry data...")
            time.sleep(0.5)

        print("Odometry data received. Starting movement...")

        start_x = self.current_position['x']
        start_y = self.current_position['y']
        self.running = True  

        twist_msg = {
            'linear': {'x': speed, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }

        while self.running:
            self.vel_pub.publish(roslibpy.Message(twist_msg))
            time.sleep(0.1)

            if self.current_position is None:
                continue

            dx = self.current_position['x'] - start_x
            dy = self.current_position['y'] - start_y
            traveled_distance = (dx ** 2 + dy ** 2) ** 0.5

            print(f"Traveled Distance: {traveled_distance:.3f} meters (Target: {distance} meters)")

            if traveled_distance >= distance:
                print("Target distance reached. Stopping robot.")
                break

        # Stop the robot after reaching the distance
        stop_msg = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        self.vel_pub.publish(roslibpy.Message(stop_msg))
        print(f"Robot stopped after traveling {traveled_distance:.2f} meters.")

    def turn_90_degrees(self, speed=0.5):
        """Turns the robot 90 degrees to the right (clockwise) using odometry feedback."""
        if self.current_yaw is None:
            print("Waiting for odometry data...")
            return
        
        start_yaw = self.current_yaw
        target_yaw = start_yaw - math.radians(88)  # 90-degree right turn (CW)
        target_yaw = self.normalize_angle(target_yaw)

        twist_msg = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': -speed}  # Negative for clockwise rotation
        }

        self.running = True
        while self.running:
            self.vel_pub.publish(roslibpy.Message(twist_msg))
            time.sleep(0.1)

            if self.current_yaw is None:
                continue

            yaw_error = self.shortest_angle_difference(self.current_yaw, target_yaw)
            print(f"Turning... Current Yaw: {math.degrees(self.current_yaw):.1f}°, Target: {math.degrees(target_yaw):.1f}°, Error: {math.degrees(yaw_error):.1f}°")

            if abs(yaw_error) < math.radians(2):  # 2-degree tolerance
                break

        # Stop rotation
        stop_msg = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        self.vel_pub.publish(roslibpy.Message(stop_msg))
        print(f"Completed 90-degree turn. Final Yaw: {math.degrees(self.current_yaw):.1f}°")

    def move_and_turn(self):
        """Drives 2 meters forward and then turns 90 degrees right."""
        print("Starting movement sequence: Drive 2m -> Turn 90°")
        self.drive_straight(speed=0.2, distance=2)
        time.sleep(1)  # Short delay before turning
        self.turn_90_degrees(speed=0.5)
        print("Movement sequence complete!")

    def start(self):
        """Start the autoMode operations."""
        self.odom_sub.subscribe(self.odom_callback)

    def stop(self):
        """Unsubscribe from topics and stop operations."""
        self.odom_sub.unsubscribe()
        self.running = False  # Stop any running processes

if __name__ == '__main__':
    controller = robotController()
    controller.start()
    
    # Move forward
    print("🚀 Moving forward...")
    controller.drive_straight(speed=0.2, distance=2)
    time.sleep(1)  # Small delay before turning
    controller.turn_90_degrees(speed=0.5)
    time.sleep(1) 
    controller.drive_straight(speed=0.2, distance=0.3)
    time.sleep(1)  # Small delay before turning
    controller.turn_90_degrees(speed=0.5)
    time.sleep(1)
    controller.drive_straight(speed=0.2, distance=2)
    controller.stop()
