import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .NPO import NPOObserver


class NPOObserverNode(Node):

    def __init__(self):
        super().__init__('npo_node')

        self.obs = NPOObserver()

        self.eta = np.zeros(3)
        self.nu  = np.zeros(3)
        self.tau = np.zeros(3)
        self.got_eta = False
        self.initialized = False

        self.create_subscription(
            Float32MultiArray,
            '/tmr4243/state/eta',
            self.cb_eta,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/tau_cmd',
            self.cb_tau,
            10
        )

        self.pub_eta_hat = self.create_publisher(
            Float32MultiArray,
            '/eta_hat',
            10
        )

        self.pub_nu_hat = self.create_publisher(
            Float32MultiArray,
            '/nu_hat',
            10
        )

        self.pub_b_hat = self.create_publisher(
            Float32MultiArray,
            '/b_hat',
            10
        )

        self.dt = 0.01
        self.create_timer(self.dt, self.step)


    def cb_eta(self, msg):
        x = float(msg.data[0])
        y = float(msg.data[1])
        psi = self.obs.wrap_angle(float(msg.data[2]))
        self.eta = np.array([x, y, psi], dtype=float)
        self.got_eta = True
        
    def cb_tau(self, msg):
        self.tau = np.array(msg.data[:3], dtype=float)

    def step(self):
        if (not self.initialized) and self.got_eta:
            self.obs.reset(self.eta, self.nu)
            self.initialized = True

        if not self.initialized:
            return

        eta_hat, nu_hat, b_hat = self.obs.step(self.dt, self.eta, self.tau)

        # Publish eta_hat
        msg_eta_hat = Float32MultiArray()
        msg_eta_hat.data = [float(eta_hat[0]), float(eta_hat[1]), float(eta_hat[2])]
        self.pub_eta_hat.publish(msg_eta_hat)

        # Publish nu_hat
        msg_nu_hat = Float32MultiArray()
        msg_nu_hat.data = [float(nu_hat[0]), float(nu_hat[1]), float(nu_hat[2])]
        self.pub_nu_hat.publish(msg_nu_hat)

        # Publish b_hat
        msg_b_hat = Float32MultiArray()
        msg_b_hat.data = [float(b_hat[0]), float(b_hat[1]), float(b_hat[2])]
        self.pub_b_hat.publish(msg_b_hat)

def main():
    rclpy.init()
    node = NPOObserverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()