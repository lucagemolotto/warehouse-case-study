#!/usr/bin/env python3
"""Simulated battery for one robot, published like a real TurtleBot3.

Charge drains with the distance driven (from odom) plus a small idle drain
per second of sim time, and is published as sensor_msgs/BatteryState on
`battery_state` (percentage in [0, 1], as the message defines it).

Parameters:
  range_m          distance a full charge lasts                     (200.0)
  idle_per_s       fraction of a full charge lost per idle second   (0.0)
  initial          starting charge, 0-1                             (1.0)
  rate_hz          publish rate                                     (1.0)

Set the charge from outside (experiments, a charging dock) by publishing
std_msgs/Float32 in [0, 1] on `battery_set`.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

FULL_VOLTAGE, EMPTY_VOLTAGE = 12.4, 11.0  # TB3 LiPo 3S, roughly


class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        self.range_m = self.declare_parameter('range_m', 200.0).value
        self.idle_per_s = self.declare_parameter('idle_per_s', 0.0).value
        self.charge = self.declare_parameter('initial', 1.0).value
        rate_hz = self.declare_parameter('rate_hz', 1.0).value

        self.last_xy = None
        self.last_t = None
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Float32, 'battery_set', self.on_set, 10)
        self.create_timer(1.0 / rate_hz, self.publish)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.drain(math.dist(self.last_xy, (p.x, p.y)) / self.range_m)
        self.last_xy = (p.x, p.y)

    def on_set(self, msg):
        self.charge = min(max(msg.data, 0.0), 1.0)
        self.get_logger().info(f'charge set to {self.charge:.3f}')

    def drain(self, amount):
        self.charge = max(self.charge - amount, 0.0)

    def publish(self):
        now = self.get_clock().now()
        if self.last_t is not None:
            self.drain(self.idle_per_s * (now - self.last_t).nanoseconds * 1e-9)
        self.last_t = now

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.percentage = float(self.charge)
        msg.voltage = EMPTY_VOLTAGE + (FULL_VOLTAGE - EMPTY_VOLTAGE) * self.charge
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(BatterySim())


if __name__ == '__main__':
    main()
