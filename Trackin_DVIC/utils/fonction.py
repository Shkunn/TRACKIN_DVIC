from typing import NamedTuple
from serial import Serial

import pyzed.sl as sl
import socket
import fcntl
import struct

class Robot_state:
    INITIALISATION  = "initialisation"
    WAITING         = "waiting"
    FOLLOWING       = "follow"
    LOST            = "lost"
    HOME            = "home"
    MANUALMODE      = "manual"
 
class Control_user: 
    STOP            = "0"
    FORWARD         = "1"
    BACKWARD        = "2"
    LEFT            = "3"
    RIGHT           = "4"
    TURN_LEFT       = "5"
    TURN_RIGHT      = "6"
    DIAG_FOR_LEFT   = "7"
    DIAG_FOR_RIGHT  = "8"
    DIAG_BACK_LEFT  = "9"
    DIAG_BACK_RIGHT = "10"

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

def check_ultrason_init(ser):
    '''
        DESCRIPTION: This function will run at the begining in
                the initialisation process and will check if 
                the ultrason sensor connection is working.
    ''' 

    data = ser.readline()
    encodor_data  = (data.decode('utf-8')).split(sep='/')
    
    if len(encodor_data) == 5:
        data_ultrasensor[0] = float(encodor_data[0])
        data_ultrasensor[1] = float(encodor_data[1])
        data_ultrasensor[2] = float(encodor_data[2])
        data_ultrasensor[3] = float(encodor_data[3])
        if(data_ultrasensor[0] != 0 or data_ultrasensor[1] != 0 or data_ultrasensor[2] != 0 or data_ultrasensor[3] != 0):
            return True

    return False
