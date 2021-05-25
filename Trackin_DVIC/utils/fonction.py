from typing import NamedTuple

import serial
import pyzed.sl as sl
import socket

class Robot_state:
    INITIALISATION = "Initialisation"
    WAITING = "Waiting"
    FOLLOWING = "Processing Global Path"
    LOST = "Automatique Mode"
    HOME = "Go to home"
    MANUALMODE = "Manual Mode"

class ParamsInit(NamedTuple):
    zed: sl.Camera
    image: sl.Mat
    pose: sl.Pose
    ser: Serial
    socket: socket.SocketType

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])