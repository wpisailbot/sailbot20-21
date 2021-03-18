import rclpy
from rclpy.node import Node
import socket
import json
from enum import Enum

#for now, publish simple string
from std_msgs.msg import String

### WIFI VARIABLES ###
TRIM_IP = '192.168.0.25' # Use this with the actual Trim Tab - it has a static IP
# TRIM_IP = '127.168.0.1' # Use this with the simulator
TRIM_PORT = 50000
BUFFER_SIZE = 50
# OWN_IP = '192.168.0.21' # This is the actual boat address
# OWN_IP = '10.0.2.15' # This is for local testing. This is whatever address you use to ssh into the board.
# OWN_IP = '192.168.0.3' # This is for local testing. This is whatever address you use to ssh into the board.
OWN_IP = '10.42.0.1'

OWN_PORT = 50051

class TRIM_STATE(int, Enum):
    MAX_LIFT_PORT: int = 0
    MAX_LIFT_STBD: int = 1
    MAX_DRAG_PORT: int = 2
    MAX_DRAG_STBD: int = 3
    MIN_LIFT: int = 4
    MANUAL: int = 5

class TeensyComms(Node):
	
    def __init__(self):
        super().__init__('teensy_comms')
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        self.s.bind((OWN_IP,TRIM_PORT))
        print("bound")
        self.s.listen(1)
        self.conn, self.addr = self.s.accept()

        #create publisher to teensy status topic
        self.teensy_status_publisher_ = self.create_publisher(String, 'teensy_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
	
        #create subscriber to teensy commands topic
        self.subscription = self.create_subscription(
            String,
            'teensy_control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        data = self.conn.recv(1024)
        if data:
            data = data.decode('utf-8')
            msg = String()
            msg.data = data
            self.teensy_status_publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

    def listener_callback(self, msg):
        self.get_logger().info('Sending to teensy: "%s"' % msg.data)
        self.conn.sendall(msg.data.encode())

def main(args=None):
    rclpy.init(args=args)

    teensy_comms = TeensyComms()

    rclpy.spin(teensy_comms)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teensy_comms.destroy_node()
  
    rclpy.shutdown()


if __name__ == '__main__':
    main()
