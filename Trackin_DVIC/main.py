from utils.fonction import *
from utils.list_ports import *
from serial import Serial

import numpy as np
import pyzed.sl as sl
import threading
import argparse

message_server = None

def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("id_name")                                                  # name to get ip adress.
    parser.add_argument("option2")
    args = parser.parse_args()

    # ZED CAMERA CONFIGURATION.        
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    init_params.coordinate_units = sl.UNIT.METER                                    # Set coordinate units.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    zed.open(init_params)
    tracking_parameters = sl.PositionalTrackingParameters()                         # enable tracking from zed.
    zed.enable_positional_tracking(tracking_parameters)
    print(f"[INIT] - open camera zed 2.")

    # ZED OBJECT CONFIGURATION.
    image = sl.Mat()                                                                # Left image from camera.
    pose = sl.Pose()  
    runtime = sl.RuntimeParameters()

    # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    port_name = get_usb()                                                           # get automaticly the micro controler.
    ser = Serial(port_name, 115200)
    commande_motor = 'e'
    ser.write(commande_motor.encode())
    print(f"[INIT] - open microcontroler on port {port_name}.")

    # INIT SERVER PARAM AND SETUP SOCKET.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((get_ip_address((bytes("{0}".format(args.id_name), 'utf-8'))), 8081))
    print(f"[INIT] - open server communication.")

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock, runtime)
    return params

def thread_listen_server(socket):
    """
        DESCRIPTION  : This thread will listen the server and transfert instruction from it.
    """
    global message_server

    while(True):
        data, addr = socket.recvfrom(1024)
        
        with lock: 
            message_server = data.decode()

def thread_slam(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                    data to other thread. It will also send camera flux to server.
    """
    zed, image, pose, ser, sock, runtime = params

    while True:
        # GET IMAGE.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        zed.get_position(pose)                                              # get position of robot.
        # print(pose.pose_data().m)

def thread_compute_command():
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                    and take descision to send to micro controler.
    """
    while(True):
        print("3")

def thread_listen_sensor(ser):
    """
        DESCRIPTION  : This thread will listen the data from ultra-son sensor from micro controler.
    """
    data_ultrasensor = np.zeros(4)

    while True:
        ser.reset_input_buffer()                                        # reset buffer to avoid delay.
        data = ser.readline()
        encodor_data  = (data.decode('utf-8')).split(sep='/')
        
        if len(encodor_data[0]) == 5:
            data_ultrasensor[0] = float(encodor_data[0])
            data_ultrasensor[1] = float(encodor_data[1])
            data_ultrasensor[2] = float(encodor_data[2])
            data_ultrasensor[3] = float(encodor_data[3])
            print(data_ultrasensor)

if __name__ == '__main__':
    params = initialize()

    # Thread listen server.
    # thread_1 = threading.Thread(target=thread_listen_server, args=(params.socket, ))
    thread_1 = threading.Thread(target=thread_listen_server, args=(params.socket,))
    thread_1.start()

    # Thread slam.
    thread_2 = threading.Thread(target=thread_slam, args=(params,))
    thread_2.start()

    # Thread compute command.
    # thread_3 = threading.Thread(target=thread_compute_command)
    # thread_3.start()

    # # Thread listen sensor.
    thread_4 = threading.Thread(target=thread_listen_sensor, args=(params.ser,))
    thread_4.start()

    thread_1.join()
    thread_2.join()
    # thread_3.join()
    thread_4.join()
