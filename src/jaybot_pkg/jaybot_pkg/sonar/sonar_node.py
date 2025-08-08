#!/usr/bin/env python3

#Eventually this node is going to have to incorporate camera data as it will detec edges
#or walls and will try to find an ideal path. This itself will decide steering as well so
#steer values benig set here is not concrete. This node maybe downsized to a simple distance ocnifguration
#node and a seperate not will be created to address full autonomy between the two sensors.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jaybot_pkg.msg import SonarStatus  # custom message
import RPi.GPIO as GPIO
import time

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')

        # BCM pin setup
        self.sonars = {
            "front": {"TRIG": 3, "ECHO": 5},
            "left": {"TRIG": 7, "ECHO": 11},
            "right": {"TRIG": 13, "ECHO": 15}
        }

        # GPIO Setup
        for config in self.sonars.values():
            GPIO.setup(config["TRIG"], GPIO.OUT)
            GPIO.setup(config["ECHO"], GPIO.IN)

        
        self.publisher_cmd = self.create_publisher(String, 'drive_command', 10)
        self.publisher_status = self.create_publisher(SonarStatus, 'sonar_status', 10)

        self.timer = self.create_timer(0.1, self.sonar_logic_loop)

        # Distance thresholds
        self.clear_thresh = 1.5
        self.caution_thresh = 0.5
        self.danger_thresh = 0.2

        # Motion params
        self.default_pwm = 100
        self.reverse_pwm = -50
        self.min_angle = 30.0
        self.max_angle = 80.0
        self.gain = 1.0  # gets increased in danger zone
        self.safe_dist = 0.5

        self.reverse_duration = 10  # in ticks (0.1s per tick = 1s)
        self.reverse_timer = 0

        # Debug vars
        self.last_zone = "CLEAR"

    def sonar_logic_loop(self):
        for label, pins in self.sonars.items():
            distance = self.read_distance(pins["TRIG"], pins["ECHO"])
            zone, pwm, angle = self.evaluate_zone(distance)

            # Construct and publish message
            status_msg = SonarStatus()
            status_msg.distance_meters = distance
            status_msg.zone = zone
            status_msg.left_pwm = pwm
            status_msg.right_pwm = pwm
            status_msg.left_steering = angle
            status_msg.right_steering = angle
            self.publisher_status.publish(status_msg)

            if zone != self.last_zone:
                self.get_logger().info(f"{label.upper()} | Zone: {zone} | PWM: {pwm} | Angle: {angle:.2f} | Dist: {distance:.2f}m")
                self.last_zone = zone

    def evaluate_zone(self, distance):

        if distance > self.clear_thresh:
            return "CLEAR", self.default_pwm, 0.0

        elif distance > self.caution_thresh:
            angle = self.max_angle * (1 - max(distance, 0.01)/self.safe_dist)
            pwm = int(self.default_pwm * (distance / self.clear_thresh))
            return "CAUTION", pwm, angle

        else:  # DANGER zone
            if self.reverse_timer == 0:
                pwm = int(self.default_pwm * (distance / self.clear_thresh))
                self.reverse_timer = self.reverse_duration
                self.gain = min(self.gain + 0.5, 4.0)  # Cap gain at 4x
                angle = self.min_angle * (self.gain * (1 - self.safe_dist / max(distance, 0.01)))
            else:
                pwm = self.reverse_pwm
                angle = self.min_angle
                self.reverse_timer -= 1

            return "DANGER", pwm, angle

    def read_distance(self, trig, echo):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        start = time.time()
        stop = time.time()

        timeout = start + 0.04  # 40ms max wait

        while GPIO.input(echo) == 0 and time.time() < timeout:
            start = time.time()
        while GPIO.input(echo) == 1 and time.time() < timeout:
            stop = time.time()

        elapsed = stop - start
        distance = (elapsed * 34300) / 2 / 100
        return min(max(distance, 0.01), 4.0)  # Clamp to 0.01–4.0 meters

def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
