import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import numpy as np

def deadzone(x, dz=0.05):
    return 0.0 if abs(x) < dz else x

def offset(raw):
    return 0.5 * (1.0 - raw)

def Rz(psi):
    return np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi),  np.cos(psi), 0],
        [0,            0,           1]
    ])

class PS5ToTau(Node):
    def __init__(self):
        super().__init__('ps5_to_tau')

        self.sub_joy = self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.sub_eta = self.create_subscription(
            Float32MultiArray,
            '/tmr4243/state/eta',
            self.cb_eta,
            10
        )

        self.pub = self.create_publisher(Float32MultiArray, '/tau_cmd', 10)

        self.psi = 0.0  # oppdateres fra eta
        self.tau_max = [2.0, 2.0, 1.0]

    def cb_eta(self, msg: Float32MultiArray):
        eta = np.array(msg.data)
        self.psi = eta[2]   # <-- dette du sa

    def cb_joy(self, msg: Joy):

        # --- JOYSTICK I BASIN FRAME ---
        JOY_X = -deadzone(msg.axes[1])
        JOY_Y = deadzone(msg.axes[0])

        l2 = offset(msg.axes[2])
        r2 = offset(msg.axes[5])
        JOY_N = deadzone(r2 - l2)

        tau_basin = np.array([JOY_X, JOY_Y, JOY_N])

        # --- ROTER TIL BODY FRAME ---
        tau_body = Rz(self.psi).T @ tau_basin

        # --- SCALE ---
        out = Float32MultiArray()
        out.data = [
            self.tau_max[0] * tau_body[0],
            self.tau_max[1] * tau_body[1],
            self.tau_max[2] * tau_body[2]
        ]

        self.pub.publish(out)

def main():
    rclpy.init()
    node = PS5ToTau()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()