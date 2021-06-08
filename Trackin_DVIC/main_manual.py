from utils.fonction import *
from utils.list_ports import *
from serial import Serial
from imutils.video import VideoStream

import math as m
import argparse
import cv2
import numpy as np
import os
import pyzed.sl as sl
import threading
import time
import socket
import imagezmq


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
courbe             = 0                                                                  # smooth turn set to 1 
sender             = None                                                               # important stuf to send stream video
human_selected     = False
id_selected        = -1

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
    global global_state, is_debug_option, fd, courbe, sender

    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("id_name", help="id_name is a joke ;)")                                                  # name to get ip adress.
    parser.add_argument("debug", help="pass debug to 1 if you want more info")
    parser.add_argument("fd", help="fd is the factor to decrease the power of our motors")
    parser.add_argument("model", help="you can choose your model : 1 for HUMAN_BODY_FAST | 2 for MULTI_CLASS_BOX_MEDIUM | 3 for MULTI_CLASS_BOX")  
    parser.add_argument("courbe", help="pass courbe to 1 if you want the robot to curve")                                                                          # you can choose your model.
    parser.add_argument("ip_server", help="ip adress of server")
    args = parser.parse_args()

    # DEBUG OPTION.
    if(args.debug == "1"):
        is_debug_option = True

    # FACTOR OPTION.
    fd = float(args.fd)

    # COURBE OPTION.
    courbe = float(args.courbe)

    # STREAM VIDEO INIT.
    sender = imagezmq.ImageSender(connect_to=f"tcp://{args.ip_server}:5555")
    
    # ZED CAMERA CONFIGURATION.        
    zed                           = None
    # init_params                   = sl.InitParameters()
    # init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    # init_params.camera_fps        = 60                             
    # init_params.coordinate_units  = sl.UNIT.METER                                    # Set coordinate units.
    # init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    
    # if(zed.open(init_params) != sl.ERROR_CODE.SUCCESS):                             
    #     print("[ERR0] Can't open camera zed 2.")
    #     exit(-1)

    # tracking_parameters = sl.PositionalTrackingParameters()                         # enable tracking from zed.
    # if(zed.enable_positional_tracking(tracking_parameters) != sl.ERROR_CODE.SUCCESS):
    #     print("[ERR0] Can't enable tracking of camera zed 2.")
    #     exit(-1)

    # print(f"[INIT] - open camera zed 2.")
    # ZED OBJECT CONFIGURATION.
    image   = None                                                           # Left image from camera.
    pose    = None 
    runtime = None 

    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param                     = None 
    # obj_param.enable_tracking     = True                                              # tracking object.

    # if(args.model == "1"):
    #     obj_param.detection_model     = sl.DETECTION_MODEL.HUMAN_BODY_FAST
    #     obj_param.enable_body_fitting = True

    # if(args.model == "2"):
    #     obj_param.detection_model     = sl.DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM

    # if(zed.enable_object_detection(obj_param) != sl.ERROR_CODE.SUCCESS):             
    #     print("[ERR0] Can't enable object detection on camera zed 2.")
    #     exit(-1)

    obj_runtime_param                                = None 
    # obj_runtime_param.detection_confidence_threshold = 40                               # 75

    # if(args.model == "2" or args.model == "3"):
    #     obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]                # Only detect Persons
        
    objects = None 
    # print(f"[INIT] - all process on zed 2 are running.")
    
    # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    port_name      = get_usb()                                                           # get automaticly the micro controler.
    ser            = Serial(port_name, 115200)
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
        #print("MESSAGE : ", message_string)
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
    global message_server, global_state, user_command, human_selected, id_selected
    
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

            # FOLLOW MODE
            parse_data = message_server.split('_')
            print(parse_data)
            if len(parse_data) == 3:
                print("JE SUIS ICI")
                # that respect the format of message.
                # (0, id, "") = desactive humain tracking.
                # (1, id, "") = active humain tracking.
                if parse_data[0] == "0":
                    human_selected = False

                if parse_data[0] == "1":
                    print("JE SUIS ICI")

                    human_selected = True
                    id_selected = int(parse_data[1])

def thread_slam(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                    data to other thread. It will also send camera flux to server.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_position, data_detection, keypoint_to_home, global_state, human_selected, id_selected

    last_time = time.time()

    hostname = socket.gethostname()

    # object = sl.ObjectData()


    while True:
        print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # # GET IMAGE.
        # zed.grab(runtime)
        # zed.retrieve_image(image, sl.VIEW.LEFT)                             
        # zed.get_position(pose)                                                          # get position of robot.

        # translation       = pose.pose_data().m[:3,3]
        # rotation          = pose.get_rotation_vector()  
        # if rotation[2] < 0:
        #     rotation[2] += 2*m.pi 
    
        # data_position[:2] = translation[:2]
        # data_position[-1] = rotation[-1]  

        # # CHECHING KEYPOINTS.
        # if global_state != Robot_state.HOME and global_state != Robot_state.RESET:
        #     keypoint_to_home = check_if_new_keypoint(keypoint_to_home, data_position[None, :], threshold=0.5, debug=True)                                                         
        
        # # CHECKING OBJECT DETECTION.
        # zed.retrieve_objects(objects, obj_runtime_param)                                # get 3D objects detection.   
        # validation, i    = get_id_nearest_humain(objects)                               # sort all object.

        # # # DRAW.
        # image_draw = image.get_data()

        # if validation:
        #     if not human_selected:
        #         # print(human_selected)
        #         index = 0
        #         for obj in objects.object_list:
        #             humain        = obj.bounding_box_2d
        #             id            = obj.id
                        
        #             point_A       = (int(humain[0][0]), int(humain[0][1]))
        #             point_B       = (int(humain[1][0]), int(humain[1][1]))
        #             point_C       = (int(humain[2][0]), int(humain[2][1]))
        #             point_D       = (int(humain[3][0]), int(humain[3][1]))
        #             color         = None
        #             if index == i:
        #                 color     = (   0,   0, 255)
        #                 data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
        #                 data_detection[1] = obj.position[0]                   # WARING! Normalement il renvoie tout le temps une valeur valide.
        #                 data_detection[2] = len(objects.object_list)
        #             else:
        #                 color     = (   0, 255,   0)
        #             image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
        #             image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
        #             image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
        #             image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)
                    
        #             font          = cv2.FONT_HERSHEY_SIMPLEX
        #             fontScale     = 2
        #             fontColor     = (255,255,255)
        #             lineType      = 2

        #             middle_x      = (point_A[0] + point_B[0]) / 2
        #             middle_y      = (point_A[1] + point_D[1]) / 2

        #             text = 'ID:' + str(id)
        #             cv2.putText(image_draw, text, 
        #                 (int(middle_x), int(middle_y)), 
        #                 font, 
        #                 fontScale,
        #                 fontColor,
        #                 lineType)
                    
        #             index += 1
        #             # print("Humain Detection: ", len(objects.object_list), " HZ: ", value)
            
        #     if human_selected and check_if_search_id_is_present(id, objects):
        #         # print(human_selected)
        #         id = id_selected
        #         objects.get_object_data_from_id(object, id)
        #         humain        = object.bounding_box_2d

        #         point_A       = (int(humain[0][0]), int(humain[0][1]))
        #         point_B       = (int(humain[1][0]), int(humain[1][1]))
        #         point_C       = (int(humain[2][0]), int(humain[2][1]))
        #         point_D       = (int(humain[3][0]), int(humain[3][1]))
        #         color         = (255, 0, 0)
        #         image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
        #         image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
        #         image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
        #         image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)

        #         font          = cv2.FONT_HERSHEY_SIMPLEX
        #         fontScale     = 2
        #         fontColor     = (255,255,255)
        #         lineType      = 2

        #         middle_x      = (point_A[0] + point_B[0]) / 2
        #         middle_y      = (point_A[1] + point_D[1]) / 2

        #         # text = 'ID:' + str(id)
        #         cv2.putText(image_draw, str(id), 
        #             (int(middle_x), int(middle_y)), 
        #             font, 
        #             fontScale,
        #             fontColor,
        #             lineType)

        #         data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
        #         data_detection[1] = object.position[0]                                                          # WARING! Normalement il renvoie tout le temps une valeur valide.
        #         data_detection[2] = len(objects.object_list)
        # else:
        #     data_detection    = np.zeros(3)  
            
        # RESET MODE.
        if(global_state == Robot_state.RESET):
            reset_transform = sl.Transform()
            # if(zed.reset_positional_tracking(reset_transform) != sl.ERROR_CODE.SUCCESS):
            #     print("[ERRO] can't reset positional tracking.")
            #     exit(-1)
            global_state = Robot_state.WAITING

        # DEBUG SHOWING WINDOWS
        # image_draw = cv2.resize(image_draw, (int(352), int(240)))
        # sender.send_image(hostname, image_draw)

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
    global data_ultrasensor, data_position, data_detection, global_state, user_command, last_command_micro, keypoint_to_home, is_debug_option, fd, courbe

    """
        INFO         : This is all local variable required for this thread.
    """
    lost_time                   = None
    param_threshold_distance    = 1.25                                                    # distance between robot and human in meters.
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
        #os.system('cls' if os.name == 'nt' else 'clear')
        np.set_printoptions(suppress = True)
        #print("Data Ultra song : ", data_ultrasensor)
        #print("Data position   : ", data_position)
        #print("Data detection  : ", data_detection)
        #print("Robot_state     : ", global_state)
        #print("Last_command_mi : ", last_command_micro)
        # print("User command    : ", user_command)
        # print("\n")
        time.sleep(0.001)

        # main algo begin at this moment.
        if(global_state == Robot_state.MANUALMODE):
            # in this mode, operator can control all robot action.
            last_command_micro = manual_mode(user_command, last_command_micro, ser)

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
        
        if len(encodor_data) == 5 and encodor_data[0] != '':
            data_ultrasensor[0] = float(encodor_data[0])
            data_ultrasensor[1] = float(encodor_data[1])
            data_ultrasensor[2] = float(encodor_data[2])
            data_ultrasensor[3] = float(encodor_data[3])
            # print(data_ultrasensor)

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

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
