#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion


class DrillshipUtilityNode(Node):
    def __init__(self):
        super().__init__('drillship_utility', namespace='drillship')

        self.thruster_names = [
            'bow_port',
            'bow_starboard',
            'stern_port',
            'stern_starboard',
        ]

        self.F_max = [1.0] * 4

        self.pubs = {}
        for name in self.thruster_names:
            topic = f'thruster/{name}/command'
            self.pubs[name] = self.create_publisher(Wrench, topic, 1)

        self.pub_eta = self.create_publisher(
            Float32MultiArray,
            '/tmr4243/state/eta',
            1
        )
        self.pub_nu = self.create_publisher(
            Float32MultiArray,
            '/tmr4243/state/nu',
            1
        )

        self.create_subscription(
            Float32MultiArray,
            '/tmr4243/command/u',
            self.u_callback,
            10
        )

        self.create_subscription(
            Odometry,
            f'measurement/odom',
            self.odom_callback,
            10
        )

        self.eta = np.zeros(3)
        self.nu = np.zeros(3)

    def u_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 8:
            self.get_logger().warn(
                f'Expected 8 values in /tmr4243/command/u, got {len(msg.data)}'
            )
            return

        u = msg.data[:4]
        alpha = msg.data[4:]

        for i, name in enumerate(self.thruster_names):
            ui = max(min(float(u[i]), 1.0), -1.0)
            ai = float(alpha[i])

            F = ui * self.F_max[i]

            wrench = Wrench()
            wrench.force.x = F * math.cos(ai)
            wrench.force.y = F * math.sin(ai)

            self.pubs[name].publish(wrench)

    def odom_callback(self, msg: Odometry):
        self.nu[0] = msg.twist.twist.linear.x
        self.nu[1] = msg.twist.twist.linear.y
        self.nu[2] = msg.twist.twist.angular.z

        self.eta[0] = msg.pose.pose.position.x
        self.eta[1] = msg.pose.pose.position.y

        _, _, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.eta[2] = yaw

        self.pub_eta.publish(Float32MultiArray(data=list(self.eta)))
        self.pub_nu.publish(Float32MultiArray(data=list(self.nu)))


def main(args=None):
    rclpy.init(args=args)
    node = DrillshipUtilityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()