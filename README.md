
# REX

New tracking robot following you wherever you go !   

TOP ANGLE                  |  FRONT                    |    TOP
:-------------------------:|:-------------------------:|:----------------------------:
![image](https://github.com/Shkunn/TRACKIN_DVIC/blob/main/Trackin_DVIC/pics/top_angle_resized.jpg?raw=true)|![image](https://github.com/Shkunn/TRACKIN_DVIC/blob/main/Trackin_DVIC/pics/front_resized.jpg?raw=true)|![image](https://github.com/Shkunn/TRACKIN_DVIC/blob/main/Trackin_DVIC/pics/top_resized.jpg?raw=true)


## VIDEO

Here is a little video of our work ! If you want to check it out juste click on teh picture

<a href="https://www.youtube.com/embed/cszdtuoA0Ps
" target="_blank"><img src="http://img.youtube.com/vi/cszdtuoA0Ps/maxresdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="640" height="360" border="10" /></a>


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


### Code Explanation

Notre démo fonctionne sur la base de 5 thread pour pouvoir échanger de la data en temps réel



## EQUIPMENT

|Name          |Tech                 |
|:-------------|:--------------------|
|Camera        |ZED 2                |
|Pont H        |Model X motor driver |
|Brain         |Jetson Nano          |
|Controler     |MEGA2560 board       |
|Shield        |Wifi Shield          |

## Aknowledgements

Used OSOYOO robot : https://osoyoo.com/2019/11/08/omni-direction-mecanum-wheel-robotic-kit-v1/
