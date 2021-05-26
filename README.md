# KTH-Neuro-Computing-Systems (Project: Robotic Head) Event Based Camera (eDVS4337) 
* Project by: Omkar Vilas Bhoite
* Advised and Guided by: Prof. Jorg Conradt & Juan Pablo Romero Bermudez

# #########################################################################################################################################################################################

# #########################################################################################################################################################################################

#### Prerequisite

###### ROS, and to install it please follow the instruction from this link "https://wiki.ros.org/ROS/Installation". "In my case I tested the code on both ROS Noetic & Melodic"

###### Also, make sure to check for the USB port and add the following in to the .cpp file. You can check to which port you are connected to using following command. (Look for "/dev/ttyACM0" in event_based_camera.cpp and change  the port as per your port name, it maybe USB* or ACM*)

```
$ ls /dev/tty*
```

###### You can check if the user is in the dialout group using the following command (Where "username" is what you have to change as per the user)

```
$ id username
```

###### If serial port is not opening then there is chance that the user is not added to the dialout group, and to add user to dialout group use the following command (this is optional, just in case you are not able to open the port)

```
$ sudo usermod -a -G dialout <username>
```

###### Now we need to make sure that we have access to the port and for that we need to give the port acccess to read and write, and to achieve that us the following command. Where the asterick in " /dev/tty* " corresponds to USB or ACM which you can verify using the above command. 

```
$ sudo chmod a+rw /dev/tty*

```
###### or you could use the following command

```
$ sudo chmod 666 /dev/tty${USB/ACM}

```

###### Install all the dependencies for the libcaer library by executing the following command.

```
$ sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev
$ sudo apt update
$ sudo apt install libserialport-dev

```


###### Install the libcaer library by executing the following commands. 

```
$ sudo add-apt-repository ppa:inivation-ppa/inivation
$ sudo apt-get update
$ sudo apt-get install libcaer-dev

```


