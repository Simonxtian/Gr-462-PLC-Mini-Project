import socket
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RFIDServerNode(Node):
    def __init__(self):
        super().__init__('rfid_server_node')

        self.publisher_ = self.create_publisher(String, 'rfid_response', 10)

        self.HOST = "0.0.0.0"
        self.PORT = 24534

        self.rfid_responses = {
            "1": 2307, "2": 4592, "3": 5797, "4": 5012,
            "5": 3815, "6": 1821, "7": 1990, "8": 3287,
            "9": 1497, "10": 2760, "11": 5155, "12": 3854,
            "13": 2605, "14": 3393, "15": 1047, "16": 3639
        }

        self.run_server()

    def run_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.HOST, self.PORT))
        server_socket.listen(5)
        self.get_logger().info(f"Server listening on {self.HOST}:{self.PORT}")

        while True:
            conn, addr = server_socket.accept()
            self.get_logger().info(f"Connected by {addr}")
            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break

                    root = ET.fromstring(data)
                    rfid_tag = root.find("RFID").text
                    response = self.rfid_responses.get(rfid_tag, 0)
                    Station_ID = root.find("StationID").text
                    Timestamp = root.find("Timestamp").text

                    # Log and publish
                    self.get_logger().info(f"RFID tag: {rfid_tag} -> Response: {response}, at station: {Station_ID}, time: {Timestamp}")
                    msg = String()
                    msg.data = f'{rfid_tag},{response},{Station_ID},{Timestamp}'
                    self.publisher_.publish(msg)

                    # Send back response
                    response_bytes = response.to_bytes(2, byteorder='little', signed=True)
                    conn.sendall(response_bytes)

            except Exception as e:
                self.get_logger().error(f"Error: {e}")
            finally:
                conn.close()
                self.get_logger().info("Client disconnected")


def main(args=None):
    rclpy.init(args=args)
    node = RFIDServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

