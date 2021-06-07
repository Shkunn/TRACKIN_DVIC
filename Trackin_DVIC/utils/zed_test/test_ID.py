import pyzed.sl as sl
import cv2
import time
import threading
import socket
import numpy as np

class Control_user: 
    FALSE            = "0"
    TRUE             = "1"

human_selected = False
message = None

"""
Define the IP address and the Port Number
"""
IP               = "172.21.72.175"
PORT             = 5000
listeningAddress = (IP, PORT)

lock             = threading.Lock()

# INIT SERVER PARAM AND SETUP SOCKET.   
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(listeningAddress)

# def thread_detection(human_selected):
def thread_detection():
    global human_selected, message

    # Create ZED objects
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    init_params.camera_fps = 60                             
    init_params.coordinate_units = sl.UNIT.METER                                    # Set coordinate units.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD 

    # Set runtime parameters
    runtime = sl.RuntimeParameters()

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        # Quit if an error occurred
        exit()
    
    # Define the Object Detection module parameters
    detection_parameters = sl.ObjectDetectionParameters()
    detection_parameters.enable_tracking    = True
    detection_parameters.detection_model    = sl.DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM                     # tracking object.
    
    # Object tracking requires camera tracking to be enabled
    if detection_parameters.enable_tracking:
        zed.enable_positional_tracking()

    print("Object Detection: Loading Module...")
    err = zed.enable_object_detection(detection_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error {}, exit program".format(err))
        zed.close()
        exit()

    # Set runtime parameter confidence to 40
    detection_parameters_runtime = sl.ObjectDetectionRuntimeParameters()
    detection_parameters_runtime.detection_confidence_threshold = 40
    detection_parameters_runtime.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons

    objects = sl.Objects()
    object = sl.ObjectData()
    image = sl.Mat()

    last_time = time.time()

    # Grab new frames and detect objects
    while True:
    
        print("THREAD :", message)
        
        value     = 1/(time.time()-last_time)
        last_time = time.time()

        # Grab an image, a RuntimeParameters object must be given to grab()
        zed.grab(runtime)
        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        # Retrieve objects
        zed.retrieve_objects(objects, detection_parameters_runtime)

        image_draw    = image.get_data()

        if not human_selected:
            # print(human_selected)
            for obj in objects.object_list:
                humain        = obj.bounding_box_2d
                id            = obj.id
                    
                point_A       = (int(humain[0][0]), int(humain[0][1]))
                point_B       = (int(humain[1][0]), int(humain[1][1]))
                point_C       = (int(humain[2][0]), int(humain[2][1]))
                point_D       = (int(humain[3][0]), int(humain[3][1]))
                color         = (0, 255, 0)
                image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)
                
                font          = cv2.FONT_HERSHEY_SIMPLEX
                fontScale     = 2
                fontColor     = (255,255,255)
                lineType      = 2

                middle_x      = (point_A[0] + point_B[0]) / 2
                middle_y      = (point_A[1] + point_D[1]) / 2

                text = 'ID:' + str(id)
                cv2.putText(image_draw, text, 
                    (int(middle_x), int(middle_y)), 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)
                
                # print("Humain Detection: ", len(objects.object_list), " HZ: ", value)
        
        if human_selected:
            # print(human_selected)
            id = int(message)
            objects.get_object_data_from_id(object, id)
            humain        = object.bounding_box_2d

            point_A       = (int(humain[0][0]), int(humain[0][1]))
            point_B       = (int(humain[1][0]), int(humain[1][1]))
            point_C       = (int(humain[2][0]), int(humain[2][1]))
            point_D       = (int(humain[3][0]), int(humain[3][1]))
            color         = (0, 255, 0)
            image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
            image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
            image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
            image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)

            font          = cv2.FONT_HERSHEY_SIMPLEX
            fontScale     = 2
            fontColor     = (255,255,255)
            lineType      = 2

            middle_x      = (point_A[0] + point_B[0]) / 2
            middle_y      = (point_A[1] + point_D[1]) / 2

            # text = 'ID:' + str(id)
            cv2.putText(image_draw, str(id), 
                (int(middle_x), int(middle_y)), 
                font, 
                fontScale,
                fontColor,
                lineType)

        cv2.WINDOW_NORMAL
        cv2.namedWindow("windows",0)
        cv2.imshow("windows", image_draw)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Disable object detection and close the camera
    zed.disable_object_detection()
    zed.close()

def thread_listen_server(lock, socket):
    """
        DESCRIPTION  : This thread will listen the server and transfert instruction from it.
    """
    global message, human_selected

    while(True):
        data, addr = socket.recvfrom(1024)
        
        with lock: 
            message_server = data.decode()

            print(type(message_server))

            message_server =    message_server.split('_')
            
            # print("ALL: ", message_server)
            # print("0  : ", message_server[0])
            # print("1  : ", message_server[1])
            # print("O: "message_server[0])

            # MODE ORDER
            if message_server[0] == Control_user.FALSE:
                human_selected = False

            if message_server[0] == Control_user.TRUE:
                human_selected = True

            message = message_server[1]


if __name__ == '__main__':
    # Thread slam.
    thread_1 = threading.Thread(target=thread_detection, args=())
    thread_1.start()

    # # Thread listen server.
    thread_2 = threading.Thread(target=thread_listen_server, args=(lock, sock))
    thread_2.start()

    thread_1.join()
    thread_2.join()

