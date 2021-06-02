from utils.fonction import *
from utils.list_ports import *
from serial import Serial

import math as m
import argparse
import cv2
import numpy as np
import os
import pyzed.sl as sl
import threading
import time
import socket


"""
    INFO    : This is all Global variable.
"""
global_state       = Robot_state.INITIALISATION
user_command       = Control_user.STOP
message_server     = None
data_ultrasensor   = np.zeros(4)
data_detection     = np.zeros(3)                                                        # format(axes y position, distance, nombre object)
data_position      = np.zeros(3)
lock               = threading.Lock()
last_command_micro = np.zeros(8)                                                        # format(moteur1FR/moteur2BR/moteur3FL/moteur4BL)
keypoint_to_home   = np.zeros((1,3))                                                    # format(format(axes y position, distance, nombre object))
is_debug_option    = False
fd                 = 0.5                                                                # factor diminution of motor power

"""
Define the IP address and the Port Number
"""
IP               = "172.21.72.168"
PORT             = 5000
listeningAddress = (IP, PORT)

# ALL GENERAL FUNCTION.

def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    global global_state, is_debug_option, fd

    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("id_name")                                                  # name to get ip adress.
    parser.add_argument("debug")
    parser.add_argument("fd")
    args = parser.parse_args()

    # DEBUG OPTION.
    if(args.debug == "1"):
        is_debug_option = True

    fd = float(args.fd)
    
    # ZED CAMERA CONFIGURATION.        
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    init_params.camera_fps = 15                             
    init_params.coordinate_units = sl.UNIT.METER                                    # Set coordinate units.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    
    if(zed.open(init_params) != sl.ERROR_CODE.SUCCESS):                             
        print("[ERR0] Can't open camera zed 2.")
        exit(-1)

    tracking_parameters = sl.PositionalTrackingParameters()                         # enable tracking from zed.
    if(zed.enable_positional_tracking(tracking_parameters) != sl.ERROR_CODE.SUCCESS):
        print("[ERR0] Can't enable tracking of camera zed 2.")
        exit(-1)

    print(f"[INIT] - open camera zed 2.")
    # ZED OBJECT CONFIGURATION.
    image = sl.Mat()                                                                # Left image from camera.
    pose = sl.Pose()  
    runtime = sl.RuntimeParameters()

    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True                                                # tracking object.
    if(zed.enable_object_detection(obj_param) != sl.ERROR_CODE.SUCCESS):             
        print("[ERR0] Can't enable object detection on camera zed 2.")
        exit(-1)

    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 75
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]                # Only detect Persons
    objects = sl.Objects()
    print(f"[INIT] - all process on zed 2 are running.")
    
    # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    port_name = get_usb()                                                           # get automaticly the micro controler.
    ser = Serial(port_name, 115200)
    commande_motor = 'e'
    if(ser.write(commande_motor.encode()) != 1):
        print(f"[ERR0] Can't call microcontroler on port {port_name}.")
        exit(-1)
    # while(check_ultrason_init(ser) == False):                                       # check if data was different of zero.
    #     print(f"[WAIT] Waiting for good ultrason data.")
    print(f"[INIT] - open microcontroler on port {port_name}.")

    # INIT SERVER PARAM AND SETUP SOCKET.   
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(listeningAddress)
    print(f"[INIT] - open server communication.")

    # CHANGE STATE.
    global_state = Robot_state.WAITING
    # global_state = Robot_state.FOLLOWING

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock, runtime, objects, obj_runtime_param)
    return params

def send_command_v2(current_command, new_command, ser):
    """
        DESCRIPTION  : Send a message to the controler if it's different.
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
        print("MESSAGE : ", message_string)
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

# ALL THREAD FUNCTION.

def thread_listen_server(lock, socket):
    """
        DESCRIPTION  : This thread will listen the server and transfert instruction from it.
    """
    global message_server, global_state, user_command
    
    # get all order available for the robot.
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

    # print("thread_listen_server WHILE")

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

def thread_slam(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                    data to other thread. It will also send camera flux to server.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_position, data_detection, keypoint_to_home, global_state

    last_time = time.time()

    while True:
        print("SLAM THREAD HZ : ", 1/(time.time() - last_time))
        last_time = time.time()

        # GET IMAGE.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        zed.get_position(pose)                                                          # get position of robot.

        translation       = pose.pose_data().m[:3,3]
        rotation          = pose.get_rotation_vector()  
        if rotation[2] < 0:
            rotation[2] += 2*m.pi 
    
        data_position[:2] = translation[:2]
        data_position[-1] = rotation[-1]  

        # CHECHING KEYPOINTS.
        if global_state != Robot_state.HOME and global_state != Robot_state.RESET:
            keypoint_to_home = check_if_new_keypoint(keypoint_to_home, data_position[None, :], threshold=0.5, debug=True)                                                         
        
        # CHECKING OBJECT DETECTION.
        zed.retrieve_objects(objects, obj_runtime_param)                                # get 3D objects detection.   
        validation, i    = get_id_nearest_humain(objects)                               # sort all object.

        # DRAW.
        image_draw = image.get_data()
        if validation:

            humain        = objects.object_list[i].bounding_box_2d
            # print("POINT_A", humain[0][0], humain[0][1])
            point_A       = (int(humain[0][0]), int(humain[0][1]))
            point_B       = (int(humain[1][0]), int(humain[1][1]))
            point_C       = (int(humain[2][0]), int(humain[2][1]))
            point_D       = (int(humain[3][0]), int(humain[3][1]))
            color         = (0, 255, 0)
            image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
            image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)  
            image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)  
            image_draw    = cv2.line(image_draw, point_D, point_A, color, 5) 

            data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
            data_detection[1] = objects.object_list[i].position[0] 
            data_detection[2] = len(objects.object_list)
        else:
            data_detection    = np.zeros(3)  

        # RESET MODE.
        if(global_state == Robot_state.RESET):
            reset_transform = sl.Transform()
            if(zed.reset_positional_tracking(reset_transform) != sl.ERROR_CODE.SUCCESS):
                print("[ERRO] can't reset positional tracking.")
                exit(-1)
            global_state = Robot_state.WAITING

        # DEBUG SHOWING WINDOWS
        
        # cv2.WINDOW_NORMAL
        # cv2.namedWindow("windows",0)
        # cv2.imshow("windows", image_draw)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

def thread_compute_command(params):
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                    and take decision to send to micro controler.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_ultrasensor, data_position, data_detection, global_state, user_command, last_command_micro, keypoint_to_home, is_debug_option, fd

    """
        INFO         : This is all local variable required for this thread.
    """
    lost_time                   = None
    param_threshold_distance    = 1                                                    # distance between robot and human in meters.
    param_plage_distance        = 0.3                                                  # threshold_distance +- plage_distance
    param_threshold_pixel_angle = 150
    threshold_angle             = 25                                                   # threshold to have to go to keypoint.
    threshold_reach_keypoint    = 0.2                                                  # threshold to say we reach keypoint.
    last_time                   = time.time()
    
    while True:
        """
            INFO     : We will see if we have access to all data in this thread.
        """
        print(f"HZ thread command : {1/(time.time()-last_time)}")
        last_time = time.time()
        # ultra son data.
        os.system('cls' if os.name == 'nt' else 'clear')
        np.set_printoptions(suppress = True)
        print("Data Ultra song : ", data_ultrasensor)
        print("Data position   : ", data_position)
        print("Data detection  : ", data_detection)
        print("Robot_state     : ", global_state)
        print("Last_command_mi : ", last_command_micro)
        # print("User command    : ", user_command)
        # print("\n")
        time.sleep(0.01)

        # main algo begin at this moment.
        if(global_state == Robot_state.MANUALMODE):
            # in this mode, operator can control all robot action.
            last_command_micro = manual_mode(user_command, last_command_micro, ser)

        if(global_state == Robot_state.FOLLOWING or global_state == Robot_state.LOST):
            # in this mode, the robot need to see humain and follow them.
            if data_detection[2] > 0:
                # we detect human ! No we need to select which command to send.
                lobal_state = Robot_state.FOLLOWING
                new_command = False

                if data_detection[0] > param_threshold_pixel_angle:
                    # need to turn right.
                    new_command = True
                    command_micro = np.array([ 1, 250*fd, 1, 250*fd, 0, 250*fd, 0, 250*fd])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if data_detection[0] < -param_threshold_pixel_angle and not new_command:
                    # need to turn left.
                    new_command = True
                    command_micro = np.array([ 0, 250*fd, 0, 250*fd, 1, 250*fd, 1, 250*fd])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if data_detection[1] > (param_threshold_distance+param_plage_distance) and not new_command:
                    # need to forward.
                    new_command = True
                    command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])

                    # # check if forward is available
                    # if(data_ultrasensor[0] > 300 or data_ultrasensor[0] == 0):
                    #     last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    # else:
                    #     # can't go forward
                    #     if((data_ultrasensor[1] == 0) and (data_ultrasensor[2] == 0)):
                    #         # if both are free, go left.
                    #         command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
                    #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    #     if(data_ultrasensor[1] > data_ultrasensor[2] and data_ultrasensor[2] != 0):
                    #         # go left
                    #         command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
                    #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    #     if(data_ultrasensor[1] < data_ultrasensor[2] and data_ultrasensor[1] != 0):
                    #         # go right
                    #         command_micro = np.array([ 0,    700, 0,    700, 0,    700, 0,    700])
                    #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    #     if((data_ultrasensor[1] < 300) and (data_ultrasensor[2] < 300) \
                    #         and (data_ultrasensor[1] != 0) and (data_ultrasensor[2] != 0)):
                    #         # block so stop
                    #         new_command = False

                if data_detection[1] < (param_threshold_distance-param_plage_distance) and not new_command:
                    # need to backward.
                    new_command = True
                    command_micro = np.array([ 1, 250*fd, 1, 250*fd, 1, 250*fd, 1, 250*fd])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if not new_command:
                    command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                
            # else:
            #     # we don't detect human.
            #     if(global_state == Robot_state.FOLLOWING):
            #         lost_time = time.time()
            #         global_state = Robot_state.LOST
            #         command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
            #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

            #     if(global_state == Robot_state.LOST and (time.time() - lost_time) > 10):
            #         # we are in Robot_state.LOST since 2.0 secondes.
            #         # so we will turn in one turn.
            #         # Turn left.
            #         command_micro      = np.array([ 0, 250*fd, 0, 250*fd, 1, 250*fd, 1, 250*fd])
            #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
            #         turn_time          = time.time()
            #         while((time.time() - turn_time) > 4):
            #             if data_detection[2] > 0:
            #                 # we detect human during turn left.
            #                 break
            #             # if no found, this line allow robot to stop for 2 secondes before turning.
            #             lost_time = time.time()
    
        if(global_state == Robot_state.HOME):

            if is_debug_option:
                print(keypoint_to_home)

            # Check if we are to home.
            if keypoint_to_home.shape[0] <= 1:
                # change state.
                global_state = Robot_state.WAITING
                command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
            else:
                # in this mode, the robot need to comeback to home.
                next_keypoint      = keypoint_to_home[-1]
                angle_direction    = calcul_vector((data_position[0], data_position[1]), (next_keypoint[0], next_keypoint[1]))
                current_angle      = data_position[2] * (180/m.pi)                                                                  # in deg.
                command_micro      = None

                if (current_angle - angle_direction) % 360 > 180:
                    distance_deg = 360 - ((current_angle - angle_direction) % 360)
                    if distance_deg > threshold_angle:
                        # turn left.
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 1, 250*fd, 1, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    else:
                        # GO forward.
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                else:
                    distance_deg = (current_angle - angle_direction) % 360
                    if distance_deg > threshold_angle:
                        # turn right.
                        command_micro = np.array([ 1, 250*fd, 1, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    else:
                        # GO forward.
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if(is_debug_option):
                    print("CURRENT ANGLE     = ", current_angle)
                    print("ANGLE DIRECTION   = ", angle_direction)
                    print("COMMAND_MICRO     = ", command_micro)

                # Check if we reach keypoint.
                if(check_if_we_reach_keypoint(data_position[None, :], next_keypoint, threshold_reach_keypoint)):
                    # we reach keypoint so delete last keypoint.
                    if(is_debug_option):
                        print("WE REACH KEYPOINT = ", keypoint_to_home[-1])

                    keypoint_to_home = np.delete(keypoint_to_home, -1, 0)

        if(global_state == Robot_state.WAITING):
            # we send stop command to engine, or we do something fun idk.
            pass
        
        if(global_state == Robot_state.RESET):
            # SPECIAL RESET MODE FOR DEBUG.
            command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
            last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
            keypoint_to_home = np.zeros((1,3))

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
    lock = threading.Lock()

    # Thread listen server.
    # thread_1 = threading.Thread(target=thread_listen_server, args=(lock, params.socket,))
    # thread_1.start()

    # Thread slam.
    thread_2 = threading.Thread(target=thread_slam, args=(params,))
    thread_2.start()

    # Thread compute command.
    # thread_3 = threading.Thread(target=thread_compute_command, args=(params,))
    # thread_3.start()

    # Thread listen sensor.
    # thread_4 = threading.Thread(target=thread_listen_sensor, args=(params.ser,))
    # thread_4.start()

    # thread_1.join()
    thread_2.join()
    # thread_3.join()
    # thread_4.join()
