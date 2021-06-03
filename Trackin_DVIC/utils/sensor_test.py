import argparse
import numpy
import time
import serial 
import struct
import numpy as np

data_ultrasensor = np.zeros(4)

ser = serial.Serial('/dev/ttyACM0', 115200)
while True:
    data = ser.readline()

    encodor_data  = (data.decode('utf-8')).split(sep='/')
    print(encodor_data)
    if len(encodor_data) == 5:
        data_ultrasensor[0] = float(encodor_data[0])
        data_ultrasensor[1] = float(encodor_data[1])
        data_ultrasensor[2] = float(encodor_data[2])
        data_ultrasensor[3] = float(encodor_data[3])
        #print(data_ultrasensor)
