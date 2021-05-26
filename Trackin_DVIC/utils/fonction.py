from typing import NamedTuple
from serial import Serial

import pyzed.sl as sl
import socket
import fcntl
import struct

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
    runtime: sl.RuntimeParameters
    objects: sl.Objects
    obj_runtime_param: sl.ObjectDetectionRuntimeParameters

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def get_id_nearest_humain(objects):
    '''
        DESCRIPTION: This function will take all object detect
                and send back the index of the nearest humain.
        INPUT      :
        *objects   > list of objects from zed sdl. 
        OUTPUT     :
        *index     > return the index not id.
    '''
    index        = 0
    i            = 0
    min_distance = -5
    is_found = False
    for obj in objects.object_list:
        if obj.position[2] > min_distance:
            index = i
            min_distance = obj.position[2]
            is_found = True
        i += 1
    
    if is_found:
        return index
    else:
        return 666
