from serial import Serial
from utils.fonction import *
from utils.list_ports import *

import argparse
import cv2
import imagezmq
import math as m
import numpy as np
import os
import psutil
import pyzed.sl as sl
import socket
import subprocess
import threading
import time


"""
    INFO    : This is all Global variable.
"""
global_state       = Robot_state.INITIALISATION
user_command       = Control_user.STOP
message_server     = None
data_ultrasensor   = np.zeros(4)
data_detection     = np.zeros(3)                        # Format(axes y position, distance, nombre object)
data_position      = np.zeros(3)
lock               = threading.Lock()
last_command_micro = np.zeros(8)                        # Format(moteur1FR/moteur2BR/moteur3FL/moteur4BL)
keypoint_to_home   = np.zeros((1,3))                    # Format(format(axes y position, distance, nombre object))
is_debug_option    = False
fd                 = 0.5                                # Factor diminution of motor power
courbe             = 0                                  # Smooth turn set to 1 
sender             = None                               # Important stuff to send stream video
human_selected     = False
id_selected        = -1
copy_image_stream  = None
new_image          = False


#region ALL GENERAL FUNCTION.

def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    global global_state, is_debug_option, fd, courbe, sender

    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("debug", help="pass debug to 1 if you want more info")
    parser.add_argument("fd", help="fd is the factor to decrease the power of our motors")
    parser.add_argument("model", help="you can choose your model : 1 for HUMAN_BODY_FAST | 2 for MULTI_CLASS_BOX_MEDIUM | 3 for MULTI_CLASS_BOX")  
    parser.add_argument("courbe", help="pass courbe to 1 if you want the robot to curve")                                                                          # you can choose your model.
    parser.add_argument("ip_server", help="ip adress of server")
    parser.add_argument("ip_brain", help="ip adress of JETSON")
    args = parser.parse_args()

    # DEBUG OPTION.
    if(args.debug == "1"):
        is_debug_option = True

    # FACTOR OPTION.
    fd     = float(args.fd)

    # COURBE OPTION.
    courbe = float(args.courbe)

    # STREAM VIDEO INIT.
    sender = imagezmq.ImageSender(connect_to=f"tcp://{args.ip_server}:5555")

    # IP OPTION.
    IP     = args.ip_brain                     # This IP use to receive messsage so you have to enter the JETSON IP
    PORT   = 5000
    listeningAddress = (IP, PORT)
    
    # ZED CAMERA CONFIGURATION.        
    zed                           = None

    # ZED OBJECT CONFIGURATION.
    image   = None
    pose    = None 
    runtime = None 

    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param                     = None 
    obj_runtime_param             = None 
    objects                       = None 
    
    # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    port_name      = get_usb()                                                        # get automaticly the micro controler.
    ser            = Serial(port_name, 115200)
    commande_motor = 'e'
    if(ser.write(commande_motor.encode()) != 1):
        print(f"[ERR0] Can't call microcontroler on port {port_name}.")
        exit(-1)

    # INIT SERVER PARAM AND SETUP SOCKET.   
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(listeningAddress)

    # CHANGE STATE.
    global_state = Robot_state.WAITING

    print("INITIALIZE FINISH")

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock, runtime, objects, obj_runtime_param)
    return params

def send_command_v2(current_command, new_command, ser):
    """
        DESCRIPTION  : Send a message to the controler if it's different.
        INPUT        :
            * current_command    = Type(Np.array)     this is the previous message sent to the micro-controller.
            * new_command        = Type(Np.array)     this is the new command to send.
            * ser                = Type(Serial)       this is the serial object.
    """
    last_command = current_command

    if(current_command[0] != new_command[0] or current_command[1] != new_command[1] or current_command[2] != new_command[2] or current_command[3] != new_command[3] or \
       current_command[4] != new_command[4] or current_command[5] != new_command[5] or current_command[6] != new_command[6] or current_command[7] != new_command[7]):
        message_string  = ""
        message_string += str(new_command[0]) + "/"
        message_string += str(new_command[1]) + "/"
        message_string += str(new_command[2]) + "/"
        message_string += str(new_command[3]) + "/"
        message_string += str(new_command[4]) + "/"
        message_string += str(new_command[5]) + "/"
        message_string += str(new_command[6]) + "/"
        message_string += str(new_command[7]) 
        ser.write(message_string.encode())
        last_command    = new_command

    return last_command

def manual_mode(user_command, last_command_micro, ser):
    """
        DESCRIPTION  : This function will manage the manual mode.
        INPUT        :
        * user_command       = Type(Control_user) this is the message from server.
        * last_command_micro = Type(Np.array)     this is the last send command to micro.
        * ser                = Type(Serial)       this is the serial object.
    """
    # FIRST. Transform Control_user en motor puissance.
    command_micro     = np.zeros(8)

    
    if(user_command == Control_user.STOP):
        command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
    
    if(user_command == Control_user.FORWARD):
        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
    
    if(user_command == Control_user.BACKWARD):
        command_micro = np.array([ 1, 250*fd, 1, 250*fd, 1, 250*fd, 1, 250*fd])
    
    if(user_command == Control_user.LEFT):
        command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
    
    if(user_command == Control_user.RIGHT):
        command_micro = np.array([ 0,    700, 0,    700, 0,    700, 0,    700])
    
    if(user_command == Control_user.TURN_LEFT):
        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 1, 250*fd, 1, 250*fd])
    
    if(user_command == Control_user.TURN_RIGHT):
        command_micro = np.array([ 1, 250*fd, 1, 250*fd, 0, 250*fd, 0, 250*fd])
    
    if(user_command == Control_user.DIAG_FOR_LEFT):
        command_micro = np.array([ 0, 250*fd, 0,   0*fd, 0,   0*fd, 0, 250*fd])
    
    if(user_command == Control_user.DIAG_FOR_RIGHT):
        command_micro = np.array([ 0,   0*fd, 0, 250*fd, 0, 250*fd, 0,   0*fd])
    
    if(user_command == Control_user.DIAG_BACK_LEFT):
        command_micro = np.array([ 0,   0*fd, 1, 250*fd, 1, 250*fd, 0,   0*fd])
    
    if(user_command == Control_user.DIAG_BACK_RIGHT):
        command_micro = np.array([ 1, 250*fd, 0,   0*fd, 0,   0*fd, 1, 250*fd])

    return send_command_v2(last_command_micro, command_micro, ser)

def kill(proc_id):
    process = psutil.Process(proc_id)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()
#endregion

#region ALL THREAD FUNCTION.

def thread_listen_server(lock, socket):
    """
        DESCRIPTION  : This thread will listen the server and transfert instruction from it.
        INPUT        :
            * lock       = Type(Threading) locking the moment when the mode is changing so that allt he code can have the same.
            * socket     = Type(Socket)    socket connection to receive the data from the website.
    """
    global message_server, global_state, user_command, human_selected, id_selected
    
    # Get all order available for the robot.
    dictionary_order = np.array([
                    ["waiting"        ],
                    ["follow"         ],
                    ["home"           ],
                    ["manual"         ],
                    ["reset"          ],
                    ["0"              ],
                    ["1"              ],
                    ["2"              ],
                    ["3"              ],
                    ["4"              ],
                    ["5"              ],
                    ["6"              ],
                    ["7"              ],
                    ["8"              ],
                    ["9"              ],
                    ["10"             ]
    ])

    while(True):
        data, addr = socket.recvfrom(1024)
        
        with lock: 
            message_server = data.decode()

            result_A = np.where(dictionary_order[:5] == message_server)
            if result_A[0].shape[0] > 0:

                # MODE ORDER
                if message_server == Robot_state.WAITING:
                    global_state = Robot_state.WAITING

                if message_server == Robot_state.FOLLOWING:
                    global_state = Robot_state.FOLLOWING

                if message_server == Robot_state.HOME:
                    global_state = Robot_state.HOME
                    
                if message_server == Robot_state.MANUALMODE:
                    global_state = Robot_state.MANUALMODE

                if message_server == Robot_state.RESET:
                    global_state = Robot_state.RESET
                    
            result_B = np.where(dictionary_order[5:] == message_server)
            if result_B[0].shape[0] > 0:

                # MANUAL COMMAND ORDER
                global_state = Robot_state.MANUALMODE
                user_command = message_server

            # FOLLOW MODE
            parse_data = message_server.split('_')
            # print(parse_data)
            if len(parse_data) == 3:
                # that respect the format of message.
                # (0, id, "") = desactive humain tracking.
                # (1, id, "") = active humain tracking.
                if parse_data[0] == "0":
                    human_selected = False

                if parse_data[0] == "1":
                    human_selected = True
                    id_selected = int(parse_data[1])

            # MAP BUILDER
            if message_server == "run":
                global_state = "RUN"

            if message_server == "stop":
                global_state = "STOP"

def thread_slam(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                    data to other thread. It will also send camera flux to server.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_position, data_detection, keypoint_to_home, global_state, human_selected, id_selected

    last_time = time.time()

    while True:
        # print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()
            
        # RESET MODE.
        if(global_state == Robot_state.RESET):
            global_state = Robot_state.WAITING

def thread_compute_command(params):
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                    and take decision to send to micro controler.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_ultrasensor, data_position, data_detection, global_state, user_command, last_command_micro, keypoint_to_home, is_debug_option, fd, courbe

    """
        INFO         : This is all local variable required for this thread.
    """
    last_time                   = time.time()
    
    while True:
        """
            INFO     : We will see if we have access to all data in this thread.
        """
        # print(f"HZ thread command : {1/(time.time()-last_time)}")
        # print("GLOBAL STATE: ",global_state)
        last_time = time.time()
        np.set_printoptions(suppress = True)
        time.sleep(0.001)

        # Main algo begin at this moment.
        if(global_state == Robot_state.MANUALMODE):
            # In this mode, operator can control all robot action.
            last_command_micro = manual_mode(user_command, last_command_micro, ser)

        if(global_state == Robot_state.WAITING):
            # We send stop command to engine, or we do something fun idk.
            pass
        
        if(global_state == Robot_state.RESET):
            # SPECIAL RESET MODE FOR DEBUG.
            command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
            last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
            keypoint_to_home = np.zeros((1,3))

def thread_listen_sensor(ser):
    """
        DESCRIPTION  : This thread will listen the data from ultra-son sensor from micro controler.
        INPUT        :
            * ser                = Type(Serial)       this is the serial object.
    """
    global data_ultrasensor

    while True:
        ser.reset_input_buffer()                                        # reset buffer to avoid delay.
        data = ser.readline()
        encodor_data  = (data.decode('utf-8')).split(sep='/')
        
        if len(encodor_data) == 5 and encodor_data[0] != '':
            data_ultrasensor[0] = float(encodor_data[0])
            data_ultrasensor[1] = float(encodor_data[1])
            data_ultrasensor[2] = float(encodor_data[2])
            data_ultrasensor[3] = float(encodor_data[3])

def thread_launch_ZED():
    global global_state

    while True:
        if global_state == "RUN":
            proc = subprocess.Popen(["./ZED_Point_Cloud_Mapping"], shell=True)
            print("ZED RECEIVE: ", global_state)
            global_state = None
            
        if global_state == "STOP":
            kill(proc.pid)
            print("ZED RECEIVE: ", global_state)
            global_state = None

#endregion

if __name__ == '__main__':

    params = initialize()
    lock = threading.Lock()

    # Thread listen server.
    thread_1 = threading.Thread(target=thread_listen_server, args=(lock, params.socket,))
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

    # Thread listen server.
    thread_5 = threading.Thread(target=thread_launch_ZED, args=())
    thread_5.start()

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
    thread_5.join()
