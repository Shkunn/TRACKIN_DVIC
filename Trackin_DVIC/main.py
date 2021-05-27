from utils.fonction import *
from utils.list_ports import *
from serial import Serial

import numpy as np
import pyzed.sl as sl
import threading
import argparse
import cv2
import time

"""
    INFO    : This is all Global variable.
"""
message_server     = None
data_ultrasensor   = np.zeros(4)
data_detection     = np.zeros(3)                                                        # format(axes y position, distance, nombre object)
data_position      = np.zeros(3)

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
    init_params.camera_fps = 15                             
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

    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True                                                # tracking object.
    zed.enable_positional_tracking()
    zed.enable_object_detection(obj_param)
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 75
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]                # Only detect Persons
    objects = sl.Objects()

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
    params = ParamsInit(zed, image, pose, ser, sock, runtime, objects, obj_runtime_param)
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
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_position, data_detection

    while True:
        # GET IMAGE.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        zed.get_position(pose)                                                          # get position of robot.

        translation      = pose.pose_data().m[:3,3]
        _,  oy,  _       = pose.get_euler_angles()       
        data_position[0] = translation[0]
        data_position[1] = translation[2]      
        data_position[2] = oy                                                           

        zed.retrieve_objects(objects, obj_runtime_param)                                # get 3D objects detection.   
        validation, i = get_id_nearest_humain(objects)                                  # sort all object.

        # DRAW.
        image_draw = image.get_data()
        if validation:

            humain        = objects.object_list[i].bounding_box_2d
            # print(humain[0][0], humain[0][1])
            point_A       = (int(humain[0][0]), int(humain[0][1]))
            point_B       = (int(humain[1][0]), int(humain[1][1]))
            point_C       = (int(humain[2][0]), int(humain[2][1]))
            point_D       = (int(humain[3][0]), int(humain[3][1]))
            color         = (0, 255, 0)
            image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
            image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)  
            image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)  
            image_draw    = cv2.line(image_draw, point_D, point_A, color, 5) 

            data_detection[0] = (int(humain[0][0]) + int(humain[1][0]))/2
            data_detection[1] = objects.object_list[i].position[2]
            data_detection[2] = len(objects.object_list)
        else:
            data_detection    = np.zeros(3)  

        # DEBUG SHOWING WINDOWS
        cv2.WINDOW_NORMAL
        cv2.namedWindow("windows",0)
        cv2.imshow("windows", image_draw)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def send_command(current_command, new_command, ser):
    """
        DESCRIPTION  : Send a message to the controler if it's different
    """
    last_command = current_command;
    
    if current_command != new_command:
        print(current_command)
        ser.write(new_command.encode())
        last_command = new_command
        print("NEW COMMAND : ", last_command)
    
    return last_command

def thread_compute_command(params):
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                    and take decision to send to micro controler.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_ultrasensor, data_position, data_detection

    while True:
        """
            INFO     : We will see if we have access to all data in this thread.
        """
        # ultra son data.
        print("Data Ultra song : ", data_ultrasensor)
        print("Data position   : ", data_position)
        print("Data detection  : ", data_detection)
        print("\n")
        time.sleep(0.1)

def thread_listen_sensor(ser):
    """
        DESCRIPTION  : This thread will listen the data from ultra-son sensor from micro controler.
    """
    global data_ultrasensor

    while True:
        ser.reset_input_buffer()                                        # reset buffer to avoid delay.
        data = ser.readline()
        encodor_data  = (data.decode('utf-8')).split(sep='/')
        
        if len(encodor_data) == 5:
            data_ultrasensor[0] = float(encodor_data[0])
            data_ultrasensor[1] = float(encodor_data[1])
            data_ultrasensor[2] = float(encodor_data[2])
            data_ultrasensor[3] = float(encodor_data[3])
            # print(data_ultrasensor)

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
    thread_3 = threading.Thread(target=thread_compute_command, args=(params,))
    thread_3.start()

    # Thread listen sensor.
    thread_4 = threading.Thread(target=thread_listen_sensor, args=(params.ser,))
    thread_4.start()

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
