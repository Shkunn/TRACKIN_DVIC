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
global_state       = Robot_state.INITIALISATION
user_command       = Control_user.STOP
message_server     = None
data_ultrasensor   = np.zeros(4)
data_detection     = np.zeros(3)                                                        # format(axes y position, distance, nombre object)
data_position      = np.zeros(3)
lock               = threading.Lock(),
last_command_micro = np.zeros(4)                                                        # format(moteur1FR/moteur2BR/moteur3FL/moteur4BL)

def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    global global_state

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
    zed.enable_object_detection(obj_param)
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
    while(check_ultrason_init(ser) == False):                                       # check if data was different of zero.
        print(f"[WAIT] Waiting for good ultrason data.")
    print(f"[INIT] - open microcontroler on port {port_name}.")

    # INIT SERVER PARAM AND SETUP SOCKET.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((get_ip_address((bytes("{0}".format(args.id_name), 'utf-8'))), 8081))
    print(f"[INIT] - open server communication.")

    # CHANGE STATE.
    global_state = Robot_state.WAITING

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock, runtime, objects, obj_runtime_param)
    return params

def send_command_v2(current_command, new_command, ser):
    """
        DESCRIPTION  : Send a message to the controler if it's different.
    """
    last_command = current_command

    if(current_command[0] != new_command[0] or current_command[1] != new_command[1] or current_command[2] != new_command[2] or current_command[3] != new_command[3]):
        message_string  = ""
        message_string += str(new_command[0]) + "/"
        message_string += str(new_command[1]) + "/"
        message_string += str(new_command[2]) + "/"
        message_string += str(new_command[3])
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
    command_micro  = np.zeros(4)

    if(user_command == Control_user.STOP):
        command_micro = np.array([   0,   0,   0,   0])
    if(user_command == Control_user.FORWARD):
        command_micro = np.array([ 200, 200, 200, 200])
    if(user_command == Control_user.BACKWARD):
        command_micro = np.array([ 800, 800, 800, 800])
    if(user_command == Control_user.LEFT):
        command_micro = np.array([ 600, 600, 600, 600])
    if(user_command == Control_user.RIGHT):
        command_micro = np.array([ 500, 500, 500, 500])
    if(user_command == Control_user.TURN_LEFT):
        command_micro = np.array([ 200, 200, 800, 800])
    if(user_command == Control_user.TURN_RIGHT):
        command_micro = np.array([ 800, 800, 200, 200])
    if(user_command == Control_user.DIAG_FOR_LEFT):
        command_micro = np.array([ 200,   0,   0, 200])
    if(user_command == Control_user.DIAG_FOR_RIGHT):
        command_micro = np.array([   0, 200, 200,   0])
    if(user_command == Control_user.DIAG_BACK_LEFT):
        command_micro = np.array([   0, 800, 800,   0])
    if(user_command == Control_user.DIAG_BACK_RIGHT):
        command_micro = np.array([ 800,   0,   0, 800])

    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

def thread_listen_server(socket):
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

            result_A = np.where(dictionary_order[:4] == message_server)
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
                    
            result_B = np.where(dictionary_order[4:] == message_server)
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

            data_detection[0] = ((int(humain[0][0]) + int(humain[1][0]))/2) - image_draw.shape[1]
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

def thread_compute_command(params):
    """
        DESCRIPTION  : This thread will analyse the data from thread_SLAM and thread_listen_sensor
                    and take decision to send to micro controler.
    """
    zed, image, pose, ser, sock, runtime, objects, obj_runtime_param = params
    global data_ultrasensor, data_position, data_detection, global_state, user_command, last_command_micro

    """
        INFO         : This is all local variable required for this thread.
    """
    lost_time                   = None
    param_threshold_distance    = -1                                                   # distance between robot and human in meters.
    param_plage_distance        = 0.1                                                  # threshold_distance +- plage_distance
    param_threshold_pixel_angle = 30

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

        # main algo begin at this moment.
        if(global_state == Robot_state.MANUALMODE):
            # in this mode, operator can control all robot action.
            manual_mode(user_command, last_command_micro, ser)

        if(global_state == Robot_state.FOLLOWING or global_state == Robot_state.LOST):
            # in this mode, the robot need to see humain and follow them.
            if data_detection[2] > 0:
                # we detect human ! No we need to select which command to send.
                lobal_state = Robot_state.FOLLOWING
                new_command = False

                if data_detection[0] > param_threshold_pixel_angle:
                    # need to turn right.
                    new_command = True
                    command_micro = np.array([ 800, 800, 200, 200])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if data_detection[0] > param_threshold_pixel_angle and not new_command:
                    # need to turn left.
                    new_command = True
                    command_micro = np.array([ 200, 200, 800, 800])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                if (data_detection[1] < (param_threshold_distance-param_plage_distance) and not new_command:
                    # need to forward.
                    new_command = True
                    command_micro = np.array([ 200, 200, 200, 200])

                    # check if forward is available
                    if(data_detection[0] > 300 or data_detection[0] == 0):
                        last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    else:
                        # can't go forward
                        if((data_detection[1] == 0) and (data_detection[2] == 0)):
                            # if both are free, go left.
                            command_micro = np.array([ 600, 600, 600, 600])
                            last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        if(data_detection[1] > data_detection[2]):
                            # go left
                            command_micro = np.array([ 600, 600, 600, 600])
                            last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        if(data_detection[1] < data_detection[2]):
                            # go right
                            command_micro = np.array([ 500, 500, 500, 500])
                            last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                        if((data_detection[1] < 300) and (data_detection[2]) < 300):
                            # block so stop
                            new_command = False

                if (data_detection[1] > (param_threshold_distance+param_plage_distance)) and not new_command:
                    # need to backward.
                    new_command = True
                    command_micro = np.array([ 800, 800, 800, 800])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)

                command_micro = np.array([   0,   0,   0,   0])
                last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                
                

            else:
                # we don't detect human.
                if(global_state == Robot_state.FOLLOWING):
                    lost_time = time.time()
                    global_state = Robot_state.LOST

                if(global_state == Robot_state.LOST and (time.time() - lost_time) > 2)
                    # we are in Robot_state.LOST since 2.0 secondes.
                    # so we will turn in one turn.
                    # Turn left.
                    command_micro      = np.array([ 200, 200, 800, 800])
                    last_command_micro = send_command_v2(last_command_micro, command_micro, ser)
                    turn_time          = time.time()
                    while((time.time() - turn_time) > 4):
                        if data_detection[2] > 0:
                            # we detect human during turn left.
                            break
                        # if no found, this line allow robot to stop for 2 secondes before turning.
                        lost_time = time.time()
    
        if(global_state == Robot_state.HOME):
            # in this mode, the robot need to comeback to home.
            pass

        if(global_state == Robot_state.WAITING):
            # we send stop command to engine, or we do something fun idk.
            pass

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
