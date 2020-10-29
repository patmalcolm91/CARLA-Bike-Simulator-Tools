"""
Contains classes and functions to easily send and receive values via UDP.
Any value or list of values which can be packed by Python's struct module can be sent.
For information on packing format strings, see https://docs.python.org/3/library/struct.html#format-characters
Author: Patrick Malcolm
"""

import socket
import struct
import time


class Server:
    """UDP server class for receiving messages"""
    def __init__(self, ip, port, fmt="!d", buffer_size=1024):
        """
        Initializes a Server instance.
        :param ip: IP address at which to establish the server. Usually localhost (127.0.0.1)
        :param port: port at which to establish the server
        :param fmt: format of messages (see https://docs.python.org/3/library/struct.html#format-characters)
        :param buffer_size: buffer size of the messages
        """
        self.ip = ip
        self.port = port
        self.format = fmt
        self.buffer_size = buffer_size
        # Set up and bind to socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)

    def get_messages(self):
        """
        Returns a list of all messages received since the last call to this function.
        :return: list of all messages received since last check.
        """
        msgs = []
        while True:
            try:
                data, addr = self.sock.recvfrom(self.buffer_size)  # buffer size is 1024 bytes
            except BlockingIOError:
                break
            else:
                msg = struct.unpack(self.format, data)
                # If only one value received, return it directly instead of as a tuple
                if len(msg) == 1:
                    msg = msg[0]
                # Add the message to the output list
                msgs.append(msg)
        return msgs


def send_message(message, ip, port, fmt="!d", broadcast=False):
    """
    Sends a UDP message to the specified address and port(s).
    :param message: Message to be sent via UDP
    :param ip: destination IP address
    :param port: destination port or list of ports
    :param fmt: byte format string (see https://docs.python.org/3/library/struct.html#format-characters)
    :param broadcast: set to True if broadcast packet
    :return: None
    :type ip: str
    :type port: int
    :type fmt: str
    :type broadcast: bool
    """
    # If message is a single value, convert it to a tuple for packing
    try:
        iter(message)
    except TypeError:
        message = (message, )
    try:
        iter(port)
    except TypeError:
        port = [port]
    # Create the socket and pack and send the message
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if broadcast:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    msg = struct.pack(fmt, *message)
    for p in port:
        sock.sendto(msg, (ip, p))


if __name__ == "__main__":
    print("Establishing test server.")
    server = Server("127.0.0.1", 30003)

    print("Waiting for messages.")
    while True:
        msgs = server.get_messages()
        print(msgs)
        time.sleep(1)
