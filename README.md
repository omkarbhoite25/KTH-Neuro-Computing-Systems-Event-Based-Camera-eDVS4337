# KTH-Neuro-Computing-Systems (Project: Robotic Head) Event Based Camera (eDVS4337) 
* Project by: Omkar Vilas Bhoite
* Advised and Guided by: Prof. Jorg Conradt & Juan Pablo Romero Bermudez

# #########################################################################################################################################################################################

# #########################################################################################################################################################################################

#### Prerequisite

###### ROS, and to install it please follow the instruction from this link "https://wiki.ros.org/ROS/Installation". "In my case I tested the code on both ROS Noetic & Melodic"



###### Install all the dependencies for the libcaer library by executing the following command.

```
$ sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev
$ sudo apt update
$ sudo apt install libserialport-dev

```

Install libserialport to access the camera, to do so execute the following command.

Install the libcaer library by executing the following commands. 

```
$ sudo add-apt-repository ppa:inivation-ppa/inivation
$ sudo apt-get update
$ sudo apt-get install libcaer-dev

```


Give the pot the permission to open and read/write:
```
$ sudo chmod 666 /dev/tty${USB/ACM}
```
