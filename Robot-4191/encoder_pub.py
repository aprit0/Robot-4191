import rclpy
from rclpy.node import Node
from gpiozero import RotaryEncoder
from std_msgs.msg import *
import time


class Encoder(Node):

    def __init__(self):
        super().__init__('encoder')
        self.publisher_ = self.create_publisher(Float32, 'encoder_rpm', 10)
        timer_period = 0.05  # seconds
        self.encoder = RotaryEncoder(a=21, b=20, max_steps=10000)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t_0 = time.time()

    def timer_callback(self):
        msg = Float32()
        t_delta = time.time() - self.t_0 
        print('t_delta ', t_delta)
        rpm = (self.encoder.steps / 48 / t_delta) * 60
        msg.data = rpm
        self.publisher_.publish(msg)
        self.encoder.steps = 0
        self.t_0 = time.time()


def main(args=None):
    rclpy.init(args=args)

    encoder_node = Encoder()

    rclpy.spin(encoder_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
