#define PY_SSIZE_T_CLEAN

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json  # Import the JSON module
import bluetooth  # Import the Bluetooth library

class MinimalPublisher(Node):

    def __init__(self, bluetooth_address):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'bluetooth_raw_value', 10)
        self.bluetooth_address = bluetooth_address
        self.bt_socket = None
        self.last_received_data = None

        # Connect to Bluetooth device
        self.connect_to_bluetooth()

    def connect_to_bluetooth(self):
        try:
            self.bt_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.bt_socket.connect((self.bluetooth_address, 1))
            self.get_logger().info('Connected to Bluetooth device at address: %s' % self.bluetooth_address)
        except Exception as e:
            self.get_logger().error('Error connecting to Bluetooth device: %s' % str(e))
            self.destroy_node()
            rclpy.shutdown()

    def receive_data(self):
        try:
            data = self.bt_socket.recv(1024)  # Adjust the buffer size as needed
            if data:
                decoded_data = data.decode()  # Ignore decoding errors
                self.get_logger().info(f'Received data: {decoded_data}')
                # Log data type
                self.get_logger().info(f'Data type: {type(decoded_data)}')

                # Check if the data has changed since the last received data
                if decoded_data != self.last_received_data:
                    # Update the last received data
                    self.last_received_data = decoded_data

                    # Convert decoded_data to a JSON object
                    json_data = self.parse_to_json(decoded_data)

                    # Publish the JSON data
                    msg = String()
                    msg.data = json_data
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().info('Data unchanged. Not publishing.')

        except Exception as e:
            self.get_logger().error('Error receiving data over Bluetooth: %s' % str(e))

    def parse_to_json(self, decoded_data):
        # Assuming decoded_data is in the format "SL:1 SM:0 SR:0"
        components = decoded_data.split()
        data_dict = {component.split(':')[0]: int(component.split(':')[1]) for component in components}
        json_data = json.dumps(data_dict)
        return json_data

def main(args=None):
    rclpy.init(args=args)

    # Specify the Bluetooth address of the device you want to connect to
    bluetooth_address = '00:21:06:08:2B:D5'  # Replace with the actual Bluetooth address

    minimal_publisher = MinimalPublisher(bluetooth_address)

    # Add a timer to regularly check for incoming data
    minimal_publisher.create_timer(0.01, minimal_publisher.receive_data)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
