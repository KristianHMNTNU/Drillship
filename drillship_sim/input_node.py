import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        self.pub_p0 = self.create_publisher(Float32MultiArray, '/path_start', 10)
        self.pub_p1 = self.create_publisher(Float32MultiArray, '/path_end', 10)
        self.pub_station = self.create_publisher(Float32MultiArray, '/station_keep_ref', 10)

        self.timer = self.create_timer(0.5, self.run)

        self.first_run = True

    def run(self):
        if self.first_run:
            print("\nCommands:")
            print("1: Set path start (p0)")
            print("2: Set path end (p1)")
            print("3: Set station keeping ref")
            print("q: quit\n")
            self.first_run = False

        cmd = input(">> ")

        if cmd == "1":
            x = float(input("x0: "))
            y = float(input("y0: "))
            self.publish(self.pub_p0, [x, y])

        elif cmd == "2":
            x = float(input("x1: "))
            y = float(input("y1: "))
            self.publish(self.pub_p1, [x, y])

        elif cmd == "3":
            x = float(input("x_ref: "))
            y = float(input("y_ref: "))
            self.publish(self.pub_station, [x, y])

        elif cmd == "q":
            rclpy.shutdown()

    def publish(self, publisher, data):
        msg = Float32MultiArray()
        msg.data = [float(d) for d in data]
        publisher.publish(msg)


def main():
    rclpy.init()
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()