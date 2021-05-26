# KTH-Neuro-Computing-Systems (Project: Robotic Head) Event Based Camera (eDVS4337) 
* Project by: Omkar Vilas Bhoite
* Advised and Guided by: Prof. Jorg Conradt & Juan Pablo Romero Bermudez

# #########################################################################################################################################################################################

# #########################################################################################################################################################################################
<img src="https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/blob/main/images/eDVS.png" width="750">


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


# #########################################################################################################################################################################################

###### Now to run the project follow the following steps.

###### Step 1. Build the project by executing the following commands.

```
$ cd camera
$ catkin_make   
$ source devel/setup.bash
```

###### Step 2. Open three terminals and execute the following command.

###### Terminal 1

```
$ roscore
```

###### Terminal 2 

```
$ rosrun event_based_camera event_based_camera
```

###### Terminal 3 

```
$ rostopic pub -1 /cam event_based_camera/Control "sos: 1"
```

# #########################################################################################################################################################################################

# #########################################################################################################################################################################################

###### Note: If you are getting error while executing the commands in terminal 2 & 3, there might be a problem of sourcing. So, you can execute the folloing command 

```
$ source devel/setup.bash
```

###### and then execute the commands in terminal 2 & 3 above.


# #########################################################################################################################################################################################


###### Note : You can also eliminate retyping the "$ source devel/setup.bash" command, by adding it in the .bashrc file. To do so you need to follow the commands
###### Open the .bashrc file. Since, I'm using vscode I will follow the following command

```
$ code ~/.bashrc
```

###### and paste the following command at the bottom of the file 

```
source ~/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/camera/devel/setup.bash
```

###### Once, it is done you need to restart the terminal, and follow the procedure to run the scripts.
