import pyzed.sl as sl
import numpy as np
import math as m
import cv2

# TEST BATCH

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

if __name__ == '__main__':
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
    
     # ACTIVE THE BATCH SYSTEM.
    object_batch = sl.ObjectsBatch()
    batch_param  = sl.BatchParameters(enable=True, id_retention_time=240, batch_duration=2.0)

    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param                               = sl.ObjectDetectionParameters()
    obj_param.enable_tracking               = True                                                # tracking object.
    obj_param.detection_model               = sl.DETECTION_MODEL.HUMAN_BODY_FAST
    obj_param.batch_parameters              = batch_param
    if(zed.enable_object_detection(obj_param) != sl.ERROR_CODE.SUCCESS):             
        print("[ERR0] Can't enable object detection on camera zed 2.")
        exit(-1)

    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 80
    # for the batch.
    
    # obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]                # Only detect Persons
    objects = sl.Objects()
    print(f"[INIT] - all process on zed 2 are running.")

    # ZED OBJECT CONFIGURATION.
    image = sl.Mat()                                                                # Left image from camera.
    pose = sl.Pose()  
    runtime = sl.RuntimeParameters()

    while True:
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        zed.get_position(pose)

        zed.retrieve_objects(objects, obj_runtime_param)                                # get 3D objects detection. 
        # validation, i    = get_id_nearest_humain(objects)                               # sort all object.

        trajectories = []
        zed.get_objects_batch(trajectories)                   # Get batch of objects
        print("Size of batch : {}".format(len(trajectories)))

        # for object in objects.object_list:
        #     print("{} {}".format(object.id, object.position))
        #     print(object.bounding_box_2d)



        # DRAW.
        image_draw = image.get_data()
        # if validation:

        #     humain        = objects.object_list[i].bounding_box_2d
        #     #print("POINT_A", humain[0][0], humain[0][1], "POINT_B", humain[1][0], humain[1][1])
        #     point_A       = (int(humain[0][0]), int(humain[0][1]))
        #     point_B       = (int(humain[1][0]), int(humain[1][1]))
        #     point_C       = (int(humain[2][0]), int(humain[2][1]))
        #     point_D       = (int(humain[3][0]), int(humain[3][1]))
        #     color         = (0, 255, 0)
        #     image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
        #     image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)  
        #     image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)  
        #     image_draw    = cv2.line(image_draw, point_D, point_A, color, 5) 

        #     middle_High   = (int(image_draw.shape[1]/2), int(0))
        #     middle_Low    = (int(image_draw.shape[1]/2), int(image_draw.shape[0]))
        #     color         = (0, 0, 255)
        #     image_draw    = cv2.line(image_draw, middle_High, middle_Low, color, 5) 

        #     middle_High   = (int((humain[0][0]+humain[1][0])/2), int(0))
        #     middle_Low    = (int((humain[0][0]+humain[1][0])/2), int(image_draw.shape[0]))
        #     color         = (0, 255, 255)
        #     image_draw    = cv2.line(image_draw, middle_High, middle_Low, color, 5) 
        #     print("VALUE WITH ZEROS CENTER: ", int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2)))

        cv2.WINDOW_NORMAL
        cv2.namedWindow("windows",0)
        cv2.imshow("windows", image_draw)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
