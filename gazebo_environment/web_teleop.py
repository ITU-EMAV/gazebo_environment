"""Drives the car from the viewer's Teleop panel, like a cruise control.

The panel's directional pad sends one direction at a time, which cannot turn a car while
it drives. This node turns the pad into car commands:
  up / down (held)     raise / lower the target speed; it stays when released. Holding
                       down stops at 0; press down again to reverse (and up again to go
                       forward after reversing)
  left / right (held)  turn at the current speed; straight again when released

The panel publishes geometry_msgs/Twist on ~/pad (up: linear.x > 0, down: linear.x < 0,
left: angular.z > 0, right: angular.z < 0). Commands go to cmd_vel only while the pad is
in use or the car is moving on its target speed, so other controllers can drive otherwise.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HOLD = 0.3  # [s] a pad message counts as "held" this long (the panel repeats it)
IDLE = 1.0  # [s] stop publishing this long after the last input once stopped


class WebTeleop(Node):
    def __init__(self):
        super().__init__("web_teleop")
        self.speed_rate = self.declare_parameter("speed_rate", 2.0).value  # [m/s per s held]
        self.brake_rate = self.declare_parameter("brake_rate", 4.0).value  # towards 0 [m/s per s]
        self.max_speed = self.declare_parameter("max_speed", 20.0).value
        self.max_reverse_speed = self.declare_parameter("max_reverse_speed", 5.0).value
        # Path curvature while left/right is held [1/m]; 0.15 is an 6.7 m radius
        self.curvature = self.declare_parameter("curvature", 0.15).value
        pad_topic = self.declare_parameter("pad_topic", "~/pad").value
        cmd_topic = self.declare_parameter("cmd_vel_topic", "cmd_vel").value

        self.publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Twist, pad_topic, self.on_pad, 10)
        self.target_speed = 0.0
        self.held = {"up": 0.0, "down": 0.0, "left": 0.0, "right": 0.0}
        # Sign of the target speed when up/down was pressed; a hold does not cross 0
        self.hold_sign = None
        self.last_input = 0.0
        self.publishing = False
        self.last_tick = self.now()
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pad(self, msg):
        now = self.now()
        self.last_input = now
        if msg.linear.x > 0:
            self.held["up"] = now
        elif msg.linear.x < 0:
            self.held["down"] = now
        if msg.angular.z > 0:
            self.held["left"] = now
        elif msg.angular.z < 0:
            self.held["right"] = now

    def tick(self):
        now = self.now()
        dt = max(0.0, now - self.last_tick)
        self.last_tick = now

        def is_held(key):
            return now - self.held[key] < HOLD

        direction = (1 if is_held("up") else 0) - (1 if is_held("down") else 0)
        if direction == 0:
            self.hold_sign = None
        else:
            if self.hold_sign is None:
                self.hold_sign = (self.target_speed > 1e-3) - (self.target_speed < -1e-3)
            slowing = self.hold_sign != 0 and direction != self.hold_sign
            rate = self.brake_rate if slowing else self.speed_rate
            speed = self.target_speed + direction * rate * dt
            if slowing and speed * self.hold_sign < 0:
                speed = 0.0  # stop at 0 within one hold
            self.target_speed = min(max(speed, -self.max_reverse_speed), self.max_speed)
        steer = (1 if is_held("left") else 0) - (1 if is_held("right") else 0)

        active = abs(self.target_speed) > 1e-3 or now - self.last_input < IDLE
        if not active:
            if self.publishing:
                self.publisher.publish(Twist())  # one stop command, then leave cmd_vel alone
                self.publishing = False
            self.target_speed = 0.0
            return

        self.publishing = True
        cmd = Twist()
        cmd.linear.x = self.target_speed
        cmd.angular.z = self.target_speed * self.curvature * steer
        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = WebTeleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
