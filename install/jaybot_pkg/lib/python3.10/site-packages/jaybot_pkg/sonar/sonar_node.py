#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jaybot_pkg.msg import SonarStatus  # custom message

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')

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
        self.gain = 1.0  # gets increased in danger
        self.safe_dist = 0.5

        self.reverse_duration = 10  # in ticks (0.1s per tick = 1s)
        self.reverse_timer = 0

        # Debug vars
        self.last_zone = "CLEAR"

    def sonar_logic_loop(self):
        # Simulated distance
        distance = self.mock_distance()

        if distance > self.clear_thresh:
            zone = "CLEAR"
            pwm = self.default_pwm
            angle = self.min_angle

        elif distance > self.caution_thresh:
            zone = "CAUTION"
            pwm = int(self.default_pwm * (distance / self.clear_thresh))
            angle = self.min_angle

        elif distance <= self.danger_thresh:
            zone = "DANGER"
            if self.reverse_timer == 0:
                pwm = self.reverse_pwm
                self.reverse_timer = self.reverse_duration
                self.gain += 0.5  # Apply new gain
                angle = self.min_angle * (self.gain * (1 - self.safe_dist / max(distance, 0.01)))
            else:
                pwm = self.reverse_pwm
                angle = self.min_angle  # Keep angle steady while reversing
            self.reverse_timer -= 1
        else:
            zone = "CAUTION"
            pwm = int(self.default_pwm * (distance / self.clear_thresh))
            angle = self.min_angle

        # Construct command string
        cmd_string = f"{pwm},{pwm},{angle},{angle}"
        msg = String()
        msg.data = cmd_string
        self.publisher_cmd.publish(msg)

        # Publish debug status
        debug = SonarStatus()
        debug.distance = float(distance)
        debug.zone = zone
        debug.left_pwm = pwm
        debug.right_pwm = pwm
        debug.left_steering = angle
        debug.right_steering = angle
        self.publisher_status.publish(debug)

        if zone != self.last_zone:
            self.get_logger().info(f"Zone: {zone} | PWM: {pwm} | Angle: {angle:.2f} | Distance: {distance:.2f}")
            self.last_zone = zone

    def mock_distance(self):
        # Replace with actual sensor reading
        import random
        return round(random.uniform(0.1, 2.0), 2)


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
