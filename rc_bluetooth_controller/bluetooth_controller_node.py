import rclpy
from rclpy.node import Node
from pynput.keyboard import Listener, Key
import threading
import bluetooth
import logging

import sys
print(sys.path)


class BluetoothControllerNode(Node):
    def __init__(self):
        super().__init__('bluetooth_controller_node')
        self.logger = self.get_logger()
        self.target_address = "00:21:06:08:2B:D5"  # Change to the actual MAC address
        # A set to keep track of keys currently pressed
        self.key_pressed = set()
        # Lock for thread-safe operations
        self.lock = threading.Lock()

        # Thread for receiving data
        self.receive_thread = threading.Thread(target=self.receive_data_thread, daemon=True)


        # Mapping arrow keys to corresponding characters
        self.key_mapping = {
            Key.up: 'F',
            Key.right: 'R',
            Key.left: 'L',
            Key.down: 'B'
        }

        # Additional key combinations
        self.key_combinations = {
            (Key.left, Key.up): 'G',
            (Key.right, Key.up): 'I',
            (Key.down, Key.right): 'J',
            (Key.down, Key.left): 'H'
        }

        # Connect to the Bluetooth HC-06 module
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def connect_to_device(self):
        try:
            self.sock.connect((self.target_address, 1))
            self.logger.info(f"Connected to the device with address: {self.target_address}")

            # Get additional information about the connected device
            device_info = bluetooth.lookup_name(self.target_address, timeout=5)
            self.logger.info(f"Device name: {device_info}")

        except bluetooth.BluetoothError as e:
            self.logger.error(f"Unable to connect to the device with address {self.target_address}: {e}")
            self.sock.close()
            exit()

    def send_data(self, data):
        with self.lock:
            self.sock.send(data)

    def receive_data_thread(self):
        while True:
            with self.lock:
                data = self.sock.recv(1024)
            if not data:
                break
            received_data = data.decode()
            self.logger.info(f"Received data: {received_data}")

    def start_receive_thread(self):
        self.receive_thread.start()

    def stop_receive_thread(self):
        self.sock.close()

    def receive_data(self):
        data = self.sock.recv(1024)
        return data.decode()

    def process_key(self, key):
        if hasattr(key, 'char'):
            return key.char.upper()
        else:
            return str(key).upper()

    def on_press(self, key):
        try:
            # Handle key combinations
            for combination, value in self.key_combinations.items():
                if key in combination and all(k in self.key_pressed for k in combination):
                    data_to_send = value
                    break
            else:
                data_to_send = self.key_mapping.get(key, self.process_key(key))

            # Check if the input is 'Q', send 'S' instead
            if data_to_send == 'Q':
                data_to_send = 'S'
            elif data_to_send == 'M':
                data_to_send = 'X'

            self.send_data(data_to_send.encode())
            self.logger.info(f"Sent data: {data_to_send}")

            # Receive data from the Bluetooth HC-06 module
            received_data = self.receive_data()
            self.logger.info(f"Received data: {received_data}")

        except Exception as e:
            self.logger.error(f"Error: {e}")

    def on_release(self, key):
        try:
            # Remove the released key from the set
            self.key_pressed.discard(key)

            if key == Key.esc:
                # Stop listener
                return False

        except Exception as e:
            self.logger.error(f"Error: {e}")

    def run_listener(self):
        try:
            with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
                listener.join()

        except KeyboardInterrupt:
            self.logger.info("Disconnected.")
        finally:
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    bluetooth_controller_node = BluetoothControllerNode()
    try:
        bluetooth_controller_node.connect_to_device()
        bluetooth_controller_node.start_receive_thread()
        bluetooth_controller_node.run_listener()
    except KeyboardInterrupt:
        pass
    finally:
        bluetooth_controller_node.stop_receive_thread()
        bluetooth_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()