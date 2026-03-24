import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .backstepping_controller import backstepping_controller


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.controller = backstepping_controller()

        self.eta_hat = np.zeros(3)
        self.nu_hat = np.zeros(3)
        self.b_hat = np.zeros(3)

        self.eta_d = np.zeros(3)
        self.eta_d_s = np.zeros(3)
        self.eta_d_ss = np.zeros(3)
        self.v_s = 0.0
        self.v_s_s = 0.0
        self.s_dot = 0.0

        self.got_eta_hat = False
        self.got_nu_hat = False
        self.got_b_hat = False
        self.got_plan = False

        self.create_subscription(
            Float32MultiArray, '/eta_hat', self.cb_eta_hat, 10
        )
        self.create_subscription(
            Float32MultiArray, '/nu_hat', self.cb_nu_hat, 10
        )
        self.create_subscription(
            Float32MultiArray, '/b_hat', self.cb_b_hat, 10
        )
        self.create_subscription(
            Float32MultiArray, '/planner_signals', self.cb_plan, 10
        )

        self.pub_tau = self.create_publisher(
            Float32MultiArray, '/tau_cmd', 10
        )

        self.dt = 0.01
        self.create_timer(self.dt, self.step)

    def cb_eta_hat(self, msg):
        self.eta_hat = np.array(msg.data[:3], dtype=float)
        self.got_eta_hat = True

    def cb_nu_hat(self, msg):
        self.nu_hat = np.array(msg.data[:3], dtype=float)
        self.got_nu_hat = True

    def cb_b_hat(self, msg):
        self.b_hat = np.array(msg.data[:3], dtype=float)
        self.got_b_hat = True

    def cb_plan(self, msg):
        data = np.array(msg.data, dtype=float)

        self.eta_d = data[0:3]
        self.eta_d_s = data[3:6]
        self.eta_d_ss = data[6:9]
        self.v_s = float(data[9])
        self.v_s_s = float(data[10])
        self.s_dot = float(data[11])

        self.got_plan = True

    def step(self):
        if not (self.got_eta_hat and self.got_nu_hat and self.got_b_hat and self.got_plan):
            return

        tau = self.controller.step(
            self.eta_hat, self.nu_hat, self.b_hat,
            self.eta_d, self.eta_d_s, self.eta_d_ss,
            self.v_s, self.v_s_s, self.s_dot
        )

        msg_tau = Float32MultiArray()
        msg_tau.data = [float(tau[0]), float(tau[1]), float(tau[2])]
        self.pub_tau.publish(msg_tau)


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()