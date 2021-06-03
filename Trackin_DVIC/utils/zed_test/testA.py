import pyzed.sl as sl
import numpy as np
import math as m

def calcul_vector(current_position, keypoint):
    """
        DESCRIPTION: calcul angle between current pose and keypoint in 
                global reference.
        OUTPUT:
            * current_position = (x, y) index of current robot position.
            * testouille       = (x, y)
        INPUT:
            * angle_degree     = (float) of vector from path
            * norm_vector      = (float) of norm from vector
    """

    x_sum = keypoint[0]-current_position[0]
    y_sum = keypoint[1]-current_position[1]
    
    angle_degree = np.arccos((x_sum)/(x_sum**2+y_sum**2)**0.5)
    if y_sum < 0:
        angle_degree = (m.pi - angle_degree) + m.pi

    return angle_degree * (180/m.pi)

if __name__ == '__main__':
    np.set_printoptions(suppress = True)
    keypoint             = np.array([0,-1])

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
    data_position = np.zeros(3)
    threshold_angle = 20

    while(True):
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        zed.get_position(pose)                                                          # get position of robot.

        translation      = pose.pose_data().m[:3,3]
        # ox,  oy,  oz     = pose.get_euler_angles(radian=True)     
        rotation         = pose.get_rotation_vector()     
        if rotation[2] < 0:
            rotation[2] += 2*m.pi

        vector_to_keypoint = calcul_vector((translation[0],translation[1]), (keypoint[0],keypoint[1]))
        current_angle      = rotation[2] * (180/m.pi)

        if (current_angle - vector_to_keypoint) % 360 > 180:
            distance_deg = 360 - ((current_angle - vector_to_keypoint) % 360)
            if distance_deg > threshold_angle:
                # turn left.
                print("TURN LEFT")
            else:
                # GO forward.
                print("GO FORWARD")
        else:
            distance_deg = (current_angle - vector_to_keypoint) % 360
            if distance_deg > threshold_angle:
                # turn right.
                print("TURN RIGHT")
            else:
                # GO forward.
                print("GO FORWARD")

        print(f"TRANSLATION = {translation} ROTATION = {rotation[2] * (180/m.pi)} VECTOR_TO_KP = {vector_to_keypoint}")  
        # data_position[0] = translation[0]
        # data_position[1] = translation[2]      
        # data_position[2] = oy

        # print("DATA_POSITION     = ", data_position)