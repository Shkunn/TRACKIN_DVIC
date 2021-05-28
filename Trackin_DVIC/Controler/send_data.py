from serial import Serial
import time
import numpy as np

arduino = Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

def write_read(FR, RR, FL, RL):
    """
        DESCRIPTION  : Send data to micro controler

        INPUTS :
            * position : position of the tracked person in relation to the camera 
                         0 : Front
                         1 : Left
                         2 : Right
            * distance : distance from the camera to the tracked person (int)
                         0 : Stop
                         1 : Go
                         2 : Backward
    """

    # arr = np.array([position, distance])
    # out_arr = np.array_str(arr)
        
    # arduino.write(out_arr.encode())
    # time.sleep(0.05)

    message = str(FR) + "/" + str(RR) + "/" + str(FL) + "/" + str(RL) 

    arduino.write(message.encode())

    return message


if __name__ == "__main__":

    counter = 0

    # while True:
    #     last    = input("Enter a position: ")
    #     counter = 0
    #     for i in range(int(last)):
    #         pos   = '0'
    #         dist  = '0'
    #         counter = write_read(pos, dist, counter)
    #         print(counter)
    #         time.sleep(0.1)

    while True:

        FR    = input("Enter a moteur FR: ")
        RR   = input("Enter a moteur RR: ")
        FL    = input("Enter a moteur FL: ")
        RL    = input("Enter a moteur RL: ")

        # dist  = input("Enter a distance: ")

        dist  = '100'

        message = write_read(FR, RR, FL, RL)
        print(message)