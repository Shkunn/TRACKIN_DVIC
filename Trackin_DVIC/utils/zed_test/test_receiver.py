# import the necessary packages
from imutils import build_montages
from datetime import datetime
import numpy as np
import imagezmq
import argparse
import cv2
import imutils
from flask import Flask, render_template, Response

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')
    
if __name__ == "__main__":
    app = Flask(__name__)

    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--prototxt", required=True,
        help="path to Caffe 'deploy' prototxt file")
    ap.add_argument("-mW", "--montageW", required=True, type=int,
        help="montage frame width")
    ap.add_argument("-mH", "--montageH", required=True, type=int,
        help="montage frame height")
    args = vars(ap.parse_args())

    # initialize the ImageHub object
    imageHub = imagezmq.ImageHub()

    # initialize the dictionary which will contain  information regarding
    # when a device was last active, then store the last time the check
    # was made was now
    lastActive = {}
    lastActiveCheck = datetime.now()
    # stores the estimated number of Pis, active checking period, and
    # calculates the duration seconds to wait before making a check to
    # see if a device was active
    ESTIMATED_NUM_PIS = 4
    ACTIVE_CHECK_PERIOD = 10
    ACTIVE_CHECK_SECONDS = ESTIMATED_NUM_PIS * ACTIVE_CHECK_PERIOD
    # assign montage width and height so we can view all incoming frames
    # in a single "dashboard"
    mW = args["montageW"]
    mH = args["montageH"]

    # start looping over all the frames
    while True:
        # receive RPi name and frame from the RPi and acknowledge
        # the receipt
        (rpiName, frame) = imageHub.recv_image()
        imageHub.send_reply(b'OK')
        # if a device is not in the last active dictionary then it means
        # that its a newly connected device
        if(rpiName not in lastActive.keys()):
            print("[INFO] receiving data from ", rpiName)
        # record the last active time for the device from which we just
        # received a frame
        lastActive[rpiName] = datetime.now()

        cv2.imshow("test", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break