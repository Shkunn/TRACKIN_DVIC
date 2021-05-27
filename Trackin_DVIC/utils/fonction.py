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
        *valid     > if list is bigger than 0.
        *index_list> position of nearest humain in objects list.
    '''
    
    if len(objects.object_list) == 0:
        return False, None
    
    min_distance = -5
    index_list   = 0
    i            = 0
    for obj in objects.object_list:
        if obj.position[2] > min_distance:
            min_distance = obj.position[2]
            index_list = i 
        i += 1

    return True, index_list        
