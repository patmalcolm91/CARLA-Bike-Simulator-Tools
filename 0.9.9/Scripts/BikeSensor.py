"""
A class to perform asynchronous communication with the bike sensor controller.

Author: Patrick Malcolm
"""

import time
import threading
import socket


ARDUINO_IP = "192.168.178.133"
ARDUINO_PORT = 5000
POLLING_INTERVAL = 0.03
SOCKET_TIMEOUT = 1


class BikeSensor:
    def __init__(self, ip=ARDUINO_IP, port=ARDUINO_PORT, polling_interval=POLLING_INTERVAL, timeout=SOCKET_TIMEOUT):
        """
        Class to handle asynchronous communication with the bike sensor controller.

        :param ip: IP address of the controller
        :param port: port of the controller
        :param polling_interval: how often to poll the sensor for data (seconds)
        :param timeout: timeout value for socket (seconds)
        :type ip: str
        :type port: int
        :type polling_interval: float
        :type timeout: float
        """
        self.address = (ip, port)
        self.interval = polling_interval
        # Set up the socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)
        # Initialize values
        self._alive = True
        self._speed = 0
        self._steering = 0
        # Start the communication thread
        self._recv_thread = threading.Thread(target=self._recv_loop, args=(threading.current_thread(),))
        self._recv_thread.start()

    def _recv_loop(self, parent_thread):
        """Communication loop"""
        while self._alive and parent_thread.is_alive():
            # Send the request packet to the sensor and wait for the response
            self.socket.sendto("foo".encode("utf-8"), self.address)  # send command to arduino
            try:
                rec_data, addr = self.socket.recvfrom(64)  # receive packet
            except socket.timeout:
                pass
            else:
                self._speed, self._steering = [float(i) for i in rec_data.decode().split("|")]
            # Sleep between sending packets
            time.sleep(self.interval)

    def get_speed_and_steering(self):
        """Retrieve the most recent values received from the sensor"""
        if not self._recv_thread.is_alive():
            self._alive = False
            self.start()
        return self._speed, self._steering

    def stop(self):
        """Kills the communication loop thread."""
        self._alive = False

    def start(self):
        """Starts a new communication loop thread."""
        if self._recv_thread.is_alive():
            raise RuntimeError("Attempting to run communication thread when one is already running.")
        self._alive = True
        self._recv_thread = threading.Thread(target=self._recv_loop, args=(threading.current_thread(),))
        self._recv_thread.start()

    def __del__(self):
        self._alive = False
        self.socket.close()


class DummyBikeSensor(BikeSensor):
    """
    A dummy bike sensor emulator for simple testing purposes.
    Ramps speed up from zero to a set value and then holds it there.
    Moves steering back and forth linearly within a set range.
    """
    def __init__(self, *args, **kwargs):
        self.speed = 0
        self.max_speed = 1600
        self.speed_step = 40
        self.steering = 0
        self.max_steering = 45
        self.steering_step = 1

    def start(self):
        pass

    def stop(self):
        pass

    def __del__(self):
        pass

    def _update_values(self):
        self.speed = min(self.speed + self.speed_step, self.max_speed)
        if self.steering_step > 1:
            self.steering = min(self.steering + self.steering_step, self.max_steering)
        else:
            self.steering = max(self.steering - self.steering_step, -self.max_steering)
        if abs(self.steering) >= self.max_steering:
            self.steering_step *= -1

    def get_speed_and_steering(self):
        speed, steering = self.speed, self.steering
        self._update_values()
        return speed, steering


if __name__ == "__main__":
    # Initialize sensor
    sensor = BikeSensor()
    # Main program loop
    while True:
        print(sensor.get_speed_and_steering())
