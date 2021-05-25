from utils.fonction import *
from utils.list_ports import *
from serial import Serial

import numpy as np
import pyzed.sl as sl
import threading
import argparse


def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("id_name")                                                  # name to get ip adress.
    parser.add_argument("option2")
    args = parser.parse_args()

    # ZED CONFIGURATION CAMERA.        
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    init_params.coordinate_units = sl.UNIT.METER                                    # Set coordinate units.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    zed.open(init_params)
    print(f"[INIT] - camera zed 2 open.")

    # ZED OBJECT CONFIGURATION.
    image = sl.Mat()                                                                # Left image from camera.
    pose = sl.Pose()  

    # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    port_name = get_usb()                                                           # get automatiqly the micro controler.
    ser = Serial(port_name, 9600)
    commande_motor = 'e'
    ser.write(commande_motor.encode())
    print(f"[INIT] - open microcontroler on port {port_name}.")

    # INIT SERVER PARAM AND SETUP SOCKET.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(sock.bind((get_ip_address((bytes("{0}".format(args.id_name), 'utf-8'))), 8081)))
    print(f"[INIT] - open server communication.")

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock)
    return params

def thread_listen_server():
    """
        DESCRIPTION  : This thread will listen the server and transfert instruction from it.
    """
    # send ping to verify server.
    pass

def thread_SLAM():
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                    data to other thread. It will also send camera flux to server.
    """
    pass

def thread_compute_command():
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                    and take descision to send to micro controler.
    """
    pass

def thread_listen_sensor():
    """
        DESCRIPTION  : This thread will listen the data from ultra-son sensor from micro controler.
    """
    pass

if __name__ == '__main__':
    params = initialize()

    # Thread listen server.
    thread_1 = threading.Thread(target=thread_listen_server, args=(params.socket, ))
    thread_1.start()

    # Thread slam.
    thread_2 = threading.Thread(target=thread_SLAM, args=(params.socket, ))
    thread_2.start()

    # Thread compute command.
    thread_3 = threading.Thread(target=thread_compute_command, args=(params.socket, ))
    thread_3.start()

    # Thread listen sensor.
    thread_4 = threading.Thread(target=thread_listen_sensor, args=(params.socket, ))
    thread_4.start()

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
