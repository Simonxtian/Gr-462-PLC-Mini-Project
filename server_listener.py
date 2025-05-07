import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RFIDListenerNode(Node):
    def __init__(self):
        super().__init__('rfid_listener_node')

        # Create a subscription to the 'rfid_response' topic
        self.subscription = self.create_subscription(
            String,
            'rfid_response',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Split the message data into parts with ,
        parts = msg.data.split(',')
        # Log the received message
        self.get_logger().info(f'Received: RFID ID: {parts[0]}, Proc time: {parts[1]}, Station: {parts[2]}, Timestamp: {parts[3]}')


def main(args=None):
    rclpy.init(args=args)
    node = RFIDListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()