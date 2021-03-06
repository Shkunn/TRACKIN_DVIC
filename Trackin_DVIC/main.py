from imutils.video import VideoStream
from serial import Serial
from utils.fonction import *
from utils.list_ports import *

import argparse
import cv2
import imagezmq
import math as m
import numpy as np
import os
import pyzed.sl as sl
import socket
import threading
import time

import firebase_admin
from firebase_admin import credentials
from firebase_admin import db   
import json

import socketio
from flask import Flask, render_template

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
data_msg           = {}


async_mode         = None
sio                = socketio.Server(async_mode='threading', cors_allowed_origins='*')
app                = Flask(__name__)

app.wsgi_app       = socketio.WSGIApp(sio, app.wsgi_app)

cred               = credentials.Certificate('FIREBASE/firebase_SDK.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://rexinterface-default-rtdb.europe-west1.firebasedatabase.app/'
})


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
    parser.add_argument("model", help="you can choose your model : 1 for HUMAN_BODY_FAST |??2 for MULTI_CLASS_BOX_MEDIUM | 3 for MULTI_CLASS_BOX")  
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

    # IP OPTION.
    IP     = args.ip_brain                     # This IP use to receive messsage so you have to enter the JETSON IP
    PORT   = 5000
    listeningAddress = (IP, PORT)

    # STREAM VIDEO INIT.
    sender = imagezmq.ImageSender(connect_to=f"tcp://{args.ip_server}:5555")
    
    # ZED CAMERA CONFIGURATION.        
    zed                           = sl.Camera()
    init_params                   = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    init_params.camera_fps        = 60                             
    init_params.coordinate_units  = sl.UNIT.METER                                   # Set coordinate units.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    
    if(zed.open(init_params) != sl.ERROR_CODE.SUCCESS):                             
        print("[ERR0] Can't open camera zed 2.")
        exit(-1)

    tracking_parameters = sl.PositionalTrackingParameters()                         # Enable tracking from zed.
    if(zed.enable_positional_tracking(tracking_parameters) != sl.ERROR_CODE.SUCCESS):
        print("[ERR0] Can't enable tracking of camera zed 2.")
        exit(-1)

    # ZED CALIBRATION.
    zed.get_camera_information().camera_configuration.calibration_parameters.left_cam

    print(f"[INIT] - open camera zed 2.")
    # ZED OBJECT CONFIGURATION.
    image   = sl.Mat()                                                              # Left image from camera.
    pose    = sl.Pose()                                                             # Pose object
    runtime = sl.RuntimeParameters()
    pymesh = sl.Mesh()                                                              # Current incremental mesh.

   
   
    # TRACKING PARAMETERS.
    tracking_parameters = sl.PositionalTrackingParameters()
    tracking_parameters.enable_area_memory = True
    zed.enable_positional_tracking(tracking_parameters)


    
    
    # SPATIAL MAPPING PARAMETERS.
    spatial_mapping_parameters = sl.SpatialMappingParameters(
        #resolution = sl.MAPPING_RESOLUTION.MEDIUM,
        #mapping_range = sl.MAPPING_RANGE.LONG,
        map_type = sl.SPATIAL_MAP_TYPE.MESH,
        use_chunk_only = True,
        max_memory_usage = 6000
    )

    mapping_parameters = sl.SpatialMappingParameters(resolution=sl.MAPPING_RESOLUTION.HIGH, use_chunk_only = True, mapping_range=sl.MAPPING_RANGE.MEDIUM, max_memory_usage = 4096*8)
    mapping_parameters.range_meter = mapping_parameters.get_range_preset(sl.MAPPING_RANGE.LONG)
    mapping_parameters.resolution_meter = 0.02

    zed.enable_spatial_mapping(mapping_parameters)


    # CLEAR FOR SAFETY?
    pymesh.clear()
    zed.enable_spatial_mapping()



    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param                     = sl.ObjectDetectionParameters()
    obj_param.enable_tracking     = True                                            # Tracking object enabled so that the objects ID won't change as long as the object stays on the ZED field of view.

    if(args.model == "1"):                                                          # Use the body tracking with is the fastest but less accurate
        obj_param.detection_model     = sl.DETECTION_MODEL.HUMAN_BODY_FAST
        obj_param.enable_body_fitting = True

    if(args.model == "2"):
        obj_param.detection_model     = sl.DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM   # Use the box detection with is fast and accurate accurate

    if(zed.enable_object_detection(obj_param) != sl.ERROR_CODE.SUCCESS):             
        print("[ERR0] Can't enable object detection on camera zed 2.")
        exit(-1)

    obj_runtime_param                                = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60

    if(args.model == "2" or args.model == "3"):
        obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]            # Only detect Persons
        
    objects = sl.Objects()
    print(f"[INIT] - all process on zed 2 are running.")
    
    # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    port_name      = get_usb()                                                      # Get automaticly the micro controler.
    ser            = Serial(port_name, 115200)
    commande_motor = 'e'
    if(ser.write(commande_motor.encode()) != 1):
        print(f"[ERR0] Can't call microcontroler on port {port_name}.")
        exit(-1)
    
    # CHECK IF MICRO-CONTROLLER SENDS DATA
    # while(check_ultrason_init(ser) == False):                                     # Check if data was different of zero.
    #     print(f"[WAIT] Waiting for good ultrason data.")

    print(f"[INIT] - open microcontroler on port {port_name}.")

    # INIT SERVER PARAM AND SETUP SOCKET.   
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(listeningAddress)
    print(f"[INIT] - open server communication.")

    # CHANGE STATE.
    global_state = Robot_state.WAITING

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock, runtime, objects, obj_runtime_param, pymesh)
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

@app.route('/')
def index():
    return render_template('latency.html')

@sio.event
def ping_from_client(sid):
    global data_msg

    print(data_msg)
    sio.emit('pong_from_server', data_msg, room=sid)

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
            print(parse_data)
            if len(parse_data) == 3:
                # that respect the format of message.
                # (0, id, "") = desactive humain tracking.
                # (1, id, "") = active humain tracking.
                if parse_data[0] == "0":
                    human_selected = False

                if parse_data[0] == "1":
                    human_selected = True
                    id_selected = int(parse_data[1])

def thread_slam(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                       data to other thread. It will also send camera flux to server.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param, pymesh = params
    global data_position, data_detection, keypoint_to_home, global_state, human_selected, id_selected, copy_image_stream, new_image, lock, data_msg

    # Init time to get the FPS 
    last_time = time.time()

    human_dict               = {}
    human_dict["Human_pose"] = {}

    # Init object for the ZED camera
    object    = sl.ObjectData()

    while True:
        print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # GET IMAGE.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        zed.get_position(pose)                                                                  # get position of robot.

        translation       = pose.pose_data().m[:3,3]
        rotation          = pose.get_rotation_vector()  
        if rotation[2] < 0:
            rotation[2] += 2*m.pi 
    
        data_position[:2] = translation[:2]
        data_position[-1] = rotation[-1]  

        # CHECHING KEYPOINTS && ADD THEM TO A LIST.
        if global_state != Robot_state.HOME and global_state != Robot_state.RESET:
            keypoint_to_home = check_if_new_keypoint(keypoint_to_home, data_position[None, :], threshold=0.5, debug=True)                                                         
        
        # CHECKING OBJECT DETECTION.
        zed.retrieve_objects(objects, obj_runtime_param)                                        # get 3D objects detection.   
        validation, i    = get_id_nearest_humain(objects)                                       # sort all object abd get the nearest.

        # CREATE THE IMAGE WE WILL SEND.
        image_draw = image.get_data()

        if validation:
            if not human_selected:
                index = 0
                for obj in objects.object_list:
                    humain        = obj.bounding_box_2d
                    id            = obj.id
                    point_A       = (int(humain[0][0]), int(humain[0][1]))
                    point_B       = (int(humain[1][0]), int(humain[1][1]))
                    point_C       = (int(humain[2][0]), int(humain[2][1]))
                    point_D       = (int(humain[3][0]), int(humain[3][1]))
                    color         = None

                    human_poseA = np.asarray([point_A])
                    human_poseC = np.asarray([point_C])

                    human_pose = np.append(human_poseA, human_poseC)

                    mat_tolist = human_pose.tolist()

                    human_dict["Human_pose"][str(id)] = mat_tolist

                    if index == i:
                        color     = (   0,   0, 255)
                        data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                        data_detection[1] = obj.position[0]
                        data_detection[2] = len(objects.object_list)
                    else:
                        color     = (   0, 255,   0)
                    
                    index += 1

                data_msg = json.dumps(human_dict)

                human_dict = {}
                human_dict["Human_pose"] = {}
            
            if human_selected and check_if_search_id_is_present(id, objects): 
                                
                for obj in objects.object_list:
                    humain        = obj.bounding_box_2d
                    id            = obj.id
                    point_A       = (int(humain[0][0]), int(humain[0][1]))
                    point_B       = (int(humain[1][0]), int(humain[1][1]))
                    point_C       = (int(humain[2][0]), int(humain[2][1]))
                    point_D       = (int(humain[3][0]), int(humain[3][1]))
                    color         = None

                    human_poseA = np.asarray([point_A])
                    human_poseC = np.asarray([point_C])

                    human_pose = np.append(human_poseA, human_poseC)

                    mat_tolist = human_pose.tolist()

                    human_dict["Human_pose"][str(id)] = mat_tolist


                
                id = id_selected
                objects.get_object_data_from_id(object, id)                         # return the object of the selected id
                humain        = object.bounding_box_2d

                data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                data_detection[1] = object.position[0]                                                          # WARING! Normalement il renvoie tout le temps une valeur valide.
                data_detection[2] = len(objects.object_list)


                human_dict = {}
                human_dict["Human_pose"] = {}
        else:
            data_detection    = np.zeros(3)  
            
        # RESET MODE.
        if(global_state == Robot_state.RESET):
            reset_transform = sl.Transform()
            if(zed.reset_positional_tracking(reset_transform) != sl.ERROR_CODE.SUCCESS):        # reset the position of the robot so reset its list of keypoints
                print("[ERRO] can't reset positional tracking.")
                exit(-1)
            global_state = Robot_state.WAITING

        # RESIZE WINDOW OTHERWISE STREAM TO SLOW 
        image_draw = cv2.resize(image_draw, (int(352), int(240)))

        #CREATE A COPY OF THE IMAGE TO SEND IT IN ANOTHER THREAD OTHERWISE LOOSE A LOT OF FPS
        with lock:
            new_image = True
        copy_image_stream = image_draw.copy()

def thread_compute_command(params):
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                       and takes a decision on what to send to the micro controler.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param, pymesh = params
    global data_ultrasensor, data_position, data_detection, global_state, user_command, last_command_micro, keypoint_to_home, is_debug_option, fd, courbe, human_selected

    """
        INFO         : This is all local variable required for this thread.
    """
    lost_time                   = None
    param_threshold_distance    = 1.25                                                 # Distance between robot and human in meters.
    param_plage_distance        = 0.3                                                  # Threshold_distance +- plage_distance.
    param_threshold_pixel_angle = 150                                                  # Threashold for the rotation of REX. 
    threshold_angle             = 25                                                   # Threshold to have to go to keypoint.
    threshold_reach_keypoint    = 0.2                                                  # Threshold to know if we reach keypoint.
    last_time                   = time.time()
    
    while True:
        """
            INFO     : We will see if we have access to all data in this thread.
        """
        print(f"HZ thread command : {1/(time.time()-last_time)}")
        last_time = time.time()
        if(is_debug_option):
            os.system('cls' if os.name == 'nt' else 'clear')
            np.set_printoptions(suppress = True)
            print("Data Ultra song : ", data_ultrasensor)
            print("Data position   : ", data_position)
            print("Data detection  : ", data_detection)
            print("Robot_state     : ", global_state)
            print("Last_command_mi : ", last_command_micro)
            print("User command    : ", user_command)
            print("\n")
        
        time.sleep(0.001)

        # Main algo begin at this moment.
        if(global_state == Robot_state.MANUALMODE):
            # In this mode, operator can control all robot action.
            last_command_micro = manual_mode(user_command, last_command_micro, ser)

        if(global_state == Robot_state.FOLLOWING or global_state == Robot_state.LOST):
            # In this mode, the robot need to see humain and follow them.
            new_command = False
            if data_detection[2] > 0:
                # We detect human ! No we need to select which command to send.
                global_state = Robot_state.FOLLOWING

                if(courbe == 1):
                    if data_detection[0] > param_threshold_pixel_angle and data_detection[1] > (param_threshold_distance+param_plage_distance):
                        # Smooth turn right
                        new_command = True
                        command_micro = np.array([ 0, 250 * fd * m.pow((1 - (data_detection[0] / 1000)), 2), 0, 250 * fd * m.pow((1 - (data_detection[0] / 1000)), 2), 0, 250 * fd * m.pow((1 + (data_detection[0] / 1000)), 2), 0, 250 * fd * m.pow((1 + (data_detection[0] / 1000)), 2)])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    
                    if data_detection[0] > param_threshold_pixel_angle and data_detection[1] >= (param_threshold_distance-param_plage_distance) and data_detection[1] <= (param_threshold_distance+param_plage_distance) and not new_command:
                        # Turn right
                        new_command = True
                        command_micro = np.array([ 1, 80, 1, 80, 0, 80, 0, 80])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    if data_detection[0] < -param_threshold_pixel_angle and data_detection[1] > (param_threshold_distance+param_plage_distance) and not new_command:
                        # Smooth turn left.
                        new_command = True
                        command_micro = np.array([ 0, 250 * fd * m.pow((1 + (-data_detection[0] / 1000)), 2), 0, 250 * fd * m.pow((1 + (-data_detection[0] / 1000)), 2), 0, 250 * fd * m.pow((1 - (-data_detection[0] / 1000)), 2), 0, 250 * fd * m.pow((1 - (-data_detection[0] / 1000)), 2)])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    if data_detection[0] < -param_threshold_pixel_angle and data_detection[1] >= (param_threshold_distance-param_plage_distance) and data_detection[1] <= (param_threshold_distance+param_plage_distance) and not new_command:
                        # Turn left
                        new_command = True
                        command_micro = np.array([ 0, 80, 0, 80, 1, 80, 1, 80])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    
                    if data_detection[1] > (param_threshold_distance+param_plage_distance) and not new_command:
                        # Need to forward.
                        # Need to check what the ultrason sensor are saying 
                        new_command = True
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        #region ULTRASON SENSOR
                        if(data_ultrasensor[0] > 300 or data_ultrasensor[0] == 0):
                            new_command = True
                            command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                            last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        else:
                            new_command = False
                            # can't go forward
                            if(data_ultrasensor[1] > 300 or data_ultrasensor[1] == 0) and (data_ultrasensor[2] > 300 or data_ultrasensor[2] == 0) and not new_command:
                                # if both are free, go left.
                                new_command = True
                                command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
                                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                                
                            if(data_ultrasensor[1] > data_ultrasensor[2] and data_ultrasensor[2] != 0 and not new_command):
                                # go left
                                new_command = True
                                command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
                                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                            if(data_ultrasensor[1] < data_ultrasensor[2] and data_ultrasensor[1] != 0 and not new_command):
                                # go right
                                new_command = True
                                command_micro = np.array([ 0,    700, 0,    700, 0,    700, 0,    700])
                                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                            if((data_ultrasensor[1] < 300) and (data_ultrasensor[2] < 300) \
                                and (data_ultrasensor[1] != 0) and (data_ultrasensor[2] != 0)):
                                # block so stop
                                new_command = False
                        #endregion

                    if data_detection[1] < (param_threshold_distance-param_plage_distance) and not new_command:
                        # Need to backward.
                        new_command = True
                        command_micro = np.array([ 1, 250*fd, 1, 250*fd, 1, 250*fd, 1, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                else:
                    if data_detection[0] > param_threshold_pixel_angle:
                        # Need to turn right.
                        new_command = True
                        command_micro = np.array([ 1, 80, 1, 80, 0, 80, 0, 80])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    if data_detection[0] < -param_threshold_pixel_angle and not new_command:
                        # Need to turn left.
                        new_command = True
                        command_micro = np.array([ 0, 80, 0, 80, 1, 80, 1, 80])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                    if data_detection[1] > (param_threshold_distance+param_plage_distance) and not new_command:
                        # Need to forward.
                        # Check if forward is available with the ultrason sensor
                        new_command = True
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        #region ULTRASON SENSOR
                        # if(data_ultrasensor[0] > 300 or data_ultrasensor[0] == 0):
                        #     new_command = True
                        #     command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        #     last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        # else:
                        #     # can't go forward
                        #     if((data_ultrasensor[1] == 0) and (data_ultrasensor[2] == 0) and not new_command):
                        #         # if both are free, go left.
                        #         new_command = True
                        #         command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
                        #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        #     if(data_ultrasensor[1] > data_ultrasensor[2] and data_ultrasensor[2] != 0 and not new_command):
                        #         # go left
                        #         new_command = True
                        #         command_micro = np.array([ 0,    600, 0,    600, 0,    600, 0,    600])
                        #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        #     if(data_ultrasensor[1] < data_ultrasensor[2] and data_ultrasensor[1] != 0 and not new_command):
                        #         # go right
                        #         new_command = True
                        #         command_micro = np.array([ 0,    700, 0,    700, 0,    700, 0,    700])
                        #         last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        #     if((data_ultrasensor[1] < 300) and (data_ultrasensor[2] < 300) \
                        #         and (data_ultrasensor[1] != 0) and (data_ultrasensor[2] != 0) and not new_command):
                        #         # block so stop
                        #         new_command = False
                        #endregion
                        
                    if data_detection[1] < (param_threshold_distance-param_plage_distance) and not new_command:
                        # Need to backward.
                        new_command = True
                        command_micro = np.array([ 1, 250*fd, 1, 250*fd, 1, 250*fd, 1, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

            if not new_command:
                command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
        
        if(global_state == Robot_state.HOME):

            if is_debug_option:
                print(keypoint_to_home)

            # Check if we are to home.
            if keypoint_to_home.shape[0] <= 1:
                # Change state.
                global_state = Robot_state.WAITING
                command_micro = np.array([ 0,   0*fd, 0,   0*fd, 0,   0*fd, 0,   0*fd])
                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
            else:
                # In this mode, the robot must return home.
                next_keypoint      = keypoint_to_home[-1]                                                                           # get the last keypoints in the list
                angle_direction    = calcul_vector((data_position[0], data_position[1]), (next_keypoint[0], next_keypoint[1]))      # calcul the angle it has to make
                current_angle      = data_position[2] * (180/m.pi)                                                                  # in degre.
                command_micro      = None

                # Calculate if the robot has to turn right or left
                if (current_angle - angle_direction) % 360 > 180:
                    distance_deg = 360 - ((current_angle - angle_direction) % 360)
                    if distance_deg > threshold_angle:
                        # Turn left.
                        command_micro = np.array([ 0, 80, 0, 80, 1, 80, 1, 80])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    else:
                        # Go forward.
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                else:
                    distance_deg = (current_angle - angle_direction) % 360
                    if distance_deg > threshold_angle:
                        # Turn right.
                        command_micro = np.array([ 1, 80, 1, 80, 0, 80, 0, 80])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    else:
                        # Go forward.
                        command_micro = np.array([ 0, 250*fd, 0, 250*fd, 0, 250*fd, 0, 250*fd])
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if(is_debug_option):
                    print("CURRENT ANGLE     = ", current_angle)
                    print("ANGLE DIRECTION   = ", angle_direction)
                    print("COMMAND_MICRO     = ", command_micro)

                # Check if we reach keypoint.
                if(check_if_we_reach_keypoint(data_position[None, :], next_keypoint, threshold_reach_keypoint)):
                    # We reach keypoint so delete last keypoint.
                    if(is_debug_option):
                        print("WE REACH KEYPOINT = ", keypoint_to_home[-1])

                    keypoint_to_home = np.delete(keypoint_to_home, -1, 0)

        if(global_state == Robot_state.WAITING):
            # We send stop command to engine, or we do something fun idk.
            human_selected = False
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

def thread_stream_image(params):
    """
        DESCRIPTION  : This thread will send the openCV image to the interface.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param, pymesh = params
    global new_image, copy_image_stream, is_debug_option

    hostname = socket.gethostname()

    last_time = time.time()

    while True:
        if new_image and copy_image_stream is not None:
            if(is_debug_option):
                print("HZ SLAM STREAM  :", 1/(time.time() - last_time))
            last_time = time.time()
            sender.send_image(hostname, copy_image_stream)
            new_image = False

def thread_pointcloud_firebase(params):

    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param, pymesh = params

    last_call = time.time()

    # Init time to get the FPS 
    last_time = time.time()

    points3D_dict              = {}
    points3D_dict["3D_points"] = {}

    limit_send_to_firebase = 0

    key = ''
    while key != 113:
        # print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # get image.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)

        #region GET MAPPING 3D POINTS

        # get position and spatial mapping state.
        zed.get_position(pose)
        zed.get_spatial_mapping_state()

        # get duration for time mapping.
        duration = time.time() - last_call  
        
        if(duration > .05):
            # -see if spatial mapping is available.
            zed.request_spatial_map_async()
        
        if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
            # -if spatial mapping is available go mapping.
            zed.retrieve_spatial_map_async(pymesh)
            last_call = time.time()

        # In background, spatial mapping will use new images, depth and pose to create and update the mesh. No specific functions are required here.
        mapping_state = zed.get_spatial_mapping_state()
        # check param.
        mapping_param = zed.get_spatial_mapping_parameters()
    
        # Print spatial mapping state and FPS
        # print("\rImages captured: {0} || {1} || {2} || {3}".format(mapping_state, mapping_param.resolution_meter, pymesh.vertices.shape[0], 1/(time.time() - last_time)))

        if pymesh.vertices.shape[0] > limit_send_to_firebase:
            mat_tolist = pymesh.vertices.tolist()

            points3D_dict["3D_points"] = mat_tolist
                        
            ref = db.reference('/')
            ref.set(points3D_dict)

            limit_send_to_firebase += 500

        #endregion

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

def thread_server():
    if sio.async_mode == 'threading':
        app.run(host='172.21.72.126 ',
                port=5001)
        
        # app.run(host='192.168.255.107 ',
        #         port=5000)        
    else:
        print('Unknown async_mode: ' + sio.async_mode)
#endregion

if __name__ == '__main__':

    params = initialize()
    lock = threading.Lock()

    # Thread listen server.
    thread_1 = threading.Thread(target=thread_listen_server       , args=(lock, params.socket,))
    thread_1.start()  
  
    # Thread slam.  
    thread_2 = threading.Thread(target=thread_slam                , args=(params,))
    thread_2.start()  
  
    # Thread compute command.  
    thread_3 = threading.Thread(target=thread_compute_command     , args=(params,))
    thread_3.start()  
  
    # Thread listen sensor.  
    thread_4 = threading.Thread(target=thread_listen_sensor       , args=(params.ser,))
    thread_4.start()  
  
    # Thread send stream image.  
    thread_5 = threading.Thread(target=thread_stream_image        , args=(params,))
    thread_5.start()  
  
    thread_6 = threading.Thread(target=thread_server              , args=())
    thread_6.start()

    thread_7 = threading.Thread(target=thread_pointcloud_firebase , args=(params,))
    thread_7.start()

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
    thread_5.join()
    thread_6.join()
    thread_7.join()