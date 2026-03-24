import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

from .SL_planner import StraightLinePathPlanner


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.eta_hat = np.zeros(3)
        self.got_eta = False

        self.s = 0.0
        self.mode = 'PATH_FOLLOWING'

        self.psi_ref = 0.0
        self.U_ref = 0.0
        self.mu = 0.2

        self.p0 = np.array([0.0, 0.0])
        self.p1 = np.array([0.0, 0.0])

        self.planner = StraightLinePathPlanner(
            self.p0, self.p1,
            psi_d=self.psi_ref,
            U_ref=self.U_ref,
            mu=self.mu
        )

        self.station_ref = np.zeros(3)

        self.create_subscription(Float32MultiArray, '/eta_hat', self.cb_eta, 10)
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.create_subscription(Float32MultiArray, '/path_start', self.cb_p0, 10)
        self.create_subscription(Float32MultiArray, '/path_end', self.cb_p1, 10)
        self.create_subscription(Float32MultiArray, '/station_keep_ref', self.cb_station_ref, 10)

        self.pub_plan = self.create_publisher(Float32MultiArray, '/planner_signals', 10)

        self.dt = 0.01
        self.create_timer(self.dt, self.step)

    def cb_eta(self, msg):
        self.eta_hat = np.array(msg.data[:3], dtype=float)
        self.got_eta = True

    def cb_p0(self, msg):
        self.p0 = np.array(msg.data[:2], dtype=float)
        self.update_planner()

    def cb_p1(self, msg):
        self.p1 = np.array(msg.data[:2], dtype=float)
        self.update_planner()

    def cb_station_ref(self, msg):
        self.station_ref[0:2] = msg.data[:2]

    def cb_joy(self, msg):
        if msg.buttons[0] == 1 and self.prev_buttons[0] == 0:
            self.mode = 'STATION_KEEPING'
            self.station_ref = self.eta_hat.copy()

        if msg.buttons[2] == 1 and self.prev_buttons[2] == 0:
            self.mode = 'PATH_FOLLOWING'

            # reset s til start (enkel løsning)
            self.s = 0.0

        self.prev_buttons = list(msg.buttons)

        L2 = msg.axes[2]
        R2 = msg.axes[5]

        psi_dot = 0.5 * (L2 - R2)
        self.psi_ref += psi_dot * self.dt

        k_u = 0.5   # tuning parameter
        U_ref_dot = k_u * msg.axes[1]
        self.U_ref += U_ref_dot * self.dt
        U_MAX = 1.0
        self.U_ref = np.clip(self.U_ref, -U_MAX, U_MAX)

        self.update_planner()

        self.get_logger().info(
        f"MODE: {self.mode} | U_ref: {self.U_ref:.2f} | psi_ref: {np.rad2deg(self.psi_ref):.2f} | Start: {self.p0} | End: {self.p1} | Station Ref: {self.station_ref[:2]}"
        )


    def update_planner(self):
        self.planner = StraightLinePathPlanner(
            self.p0, self.p1,
            psi_d=self.psi_ref,
            U_ref=self.U_ref,
            mu=self.mu
        )

    def step(self):
        if not self.got_eta:
            return

        if self.mode == 'PATH_FOLLOWING':
            eta_d, eta_d_s, eta_d_ss, v_s, v_s_s, s_dot = \
                self.planner.get_signals(self.eta_hat, self.s)

            if self.s >= 1.0 or (self.s + self.dt * s_dot) >= 1.0:
                s_dot = 0.0
                v_s = 0.0
                v_s_s = 0.0

            if self.s < 0 or (self.s + self.dt * s_dot) < 0:
                s_dot = 0.0

                v_s = max(v_s, 0.0)
                v_s_s = 0.0

            self.s += self.dt * s_dot
            self.s = np.clip(self.s, 0.0, 1.0)

        else:
            eta_d = np.array([
                self.station_ref[0],
                self.station_ref[1],
                self.psi_ref
            ])

            eta_d_s = np.zeros(3)
            eta_d_ss = np.zeros(3)

            v_s = 0.0
            v_s_s = 0.0
            s_dot = 0.0

        msg = Float32MultiArray()
        msg.data = [
            *eta_d,
            *eta_d_s,
            *eta_d_ss,
            float(v_s),
            float(v_s_s),
            float(s_dot)
        ]

        self.pub_plan.publish(msg)


def main():
    rclpy.init()
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()