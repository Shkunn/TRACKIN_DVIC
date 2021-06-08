# REX

New tracking robot following you wherever you go !   

TOP ANGLE                  |  FRONT                    |    TOP
:-------------------------:|:-------------------------:|:----------------------------:
![image](https://github.com/Shkunn/TRACKIN_DVIC/blob/main/Trackin_DVIC/pics/top_angle_resized.jpg?raw=true)|![image](https://github.com/Shkunn/TRACKIN_DVIC/blob/main/Trackin_DVIC/pics/front_resized.jpg?raw=true)|![image](https://github.com/Shkunn/TRACKIN_DVIC/blob/main/Trackin_DVIC/pics/top_resized.jpg?raw=true)



## DESCRIPTION

For now, to launch our demo you have to run the `Threadin_DVIC/main.py` on your terminal with the following arguments:

* **id_name**   : id_name is a joke
* **debug**     : if you want more information during processing 
  * 0 if you don't
  * 1 if you do
* **fd**        : factor to decrease the power of our motors (between 0 and 1)
* **model**     : you can choose your model
  * 1 for HUMAN_BODY_FAST
  * 2 for MULTI_CLASS_BOX_MEDIUM
  * 3 for MULTI_CLASS_BOX
* **courbe**    : REX can curve while turning 
  * 0 if you don't want
  * 1 if you want
* **ip_server** : ip adress from where the whebsite is launched  

**For exemple a message looks like this :** `python3 main.py wlp8s0 0 0.5 2 1 172.21.72.133`
### Code informations

REX is using a **SLAM** algorithm which allows him to locate himself in an environment. Thanks to that, he is recording his position every 50cm so thta he could return where he started whenever you want. 

We've implemented a kind of State Machine so that REX can switch to five mode:

|Mode          |Explanation               |
|:-------------|:-------------------------|
|Waiting       |stop the motors           |
|Following     |you need to select the ID on the website and then REX <br /> will follow him with a safety distance of +- 1.30 meter|
|Home          |REX uses the points he placed during his movements to <br /> return to where he began|
|Manual Mode   |You can control REX with the arrows on the website |
|Reset         |Will reset the list of points that REX created to be <br /> able to return HOME |

### Code Explanation

Our robot works with five continuous threads which are:

* **thread_listen_server** : its purpose is to retrieve messages coming from the web <br />
site to either activate one of the five modes or send data such as the messageit must <br />
send to the engine
* **thread_slam** : This thread will listen the camera zed sdk information and transfert <br /> 
data to other thread. It will also compute the human detection and then send camera flux to the server.
* **thread_compute_command** : This thread will analyse the data from thread_SLAM <br /> and
thread_listen_sensor and take decision to send to micro controler.
* **thread_listen_sensor** : REX is also equipped with four Arduino ultrason sensor <br />
aimed to control if there's obstacles around him ! This thread only receives the data 
* **thread_stream_image** : send the openCV image with the human detection on it to the <br />
website for live stream 

## EQUIPMENT

|Name          |Tech                      |
|:-------------|:-------------------------|
|Camera        |ZED 2                     |
|Pont H        |Model X motor driver      |
|Brain         |Jetson Nano               |
|Controler     |MEGA2560 board            |
|Shield        |Wifi Shield               |
|Sensor        |HC-SR04 ultrasonic sensor |
|Sensor        |Voltage Converter         |


## Aknowledgements

Used OSOYOO robot : https://osoyoo.com/2019/11/08/omni-direction-mecanum-wheel-robotic-kit-v1/
