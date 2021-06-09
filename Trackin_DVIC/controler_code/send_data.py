from serial import Serial
import time
import numpy as np

arduino = Serial(port='/dev/ttyACM1', baudrate=115200, timeout=.1)

def write_read(bf1, FR, bf2, RR, bf3, FL, bf4, RL):
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

    message = str(bf1) + "/" + str(FR) + "/" + str(bf2) + "/" + str(RR) +  "/" + str(bf3) + "/" + str(FL) + "/" + str(bf4) + "/" + str(RL) 

    arduino.write(message.encode())

    return message


if __name__ == "__main__":

    counter = 0

    while True:

        bf1   = input("Enter a moteur bf1 : ")
        FR    = input("Enter a moteur FR  : ")
        bf2   = input("Enter a moteur bf2 : ")
        RR    = input("Enter a moteur RR  : ")
        bf3   = input("Enter a moteur bf3 : ")
        FL    = input("Enter a moteur FL  : ")
        bf4   = input("Enter a moteur bf4 : ")
        RL    = input("Enter a moteur RL  : ")

        message = write_read(bf1, FR, bf2, RR, bf3, FL, bf4, RL)
        print(message)