**********************************INSTALLATION DEPENDENCIES************************************

TEENSYDUINO FOR LOAD CELL:
--install arduino software: https://www.arduino.cc/en/Main/Software

--copy teensy rules to udev folder
--download teensy software: 	https://www.pjrc.com/teensy/td_download.html
--install teensy software: 	chmod 755 TeensyduinoInstall.linux64
				./TeensyduinoInstall.linux64


ROSSERIAL FOR LOAD CELL:
--install pip: sudo apt-get install python-pip
--pip install pyserial

(from here: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
--sudo apt-get install ros-indigo-rosserial-arduino
--sudo apt-get install ros-indigo-rosserial
--cd Downloads/arduino-1.8.1/libraries
--rm -rf ros_lib
--rosrun rosserial_arduino make_libraries.py .


HEBI MOTOR MODULES:
--sudo apt-get install default-jre

--Download the gui as a jar file:
--http://docs.hebi.us/ username: hebiguest password: documentation

--Install Java 1.8: 
--https://www.liquidweb.com/kb/how-to-install-oracle-java-8-on-ubuntu-14-04-lts/
--launch Hebi GUI using java -jar


MAXON EPOS2 MOTOR CONTROLLER:
--Download Linux library:
--http://www.maxonmotorusa.com/maxon/view/product/control/Positionierung/390438?download=show

--Download FTDI drivers: http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx0.4.16.tar.gz

Link all of the .so files (https://wpirover.files.wordpress.com/2011/09/maxon-ubuntu-instructions2.pdf):
--Change your current directory to where you unzipped the FTDI files, where libftd2xx.so.0.4.16 is located 
--sudo cp libftd2xx.so.0.4.16 /usr/local/lib
--cd /usr/local/lib
--sudo ln -s libftd2xx.so.0.4.16 libftd2xx.so.0 
--sudo ln -s libftd2xx.so.0 libftd2xx.so
--cd /usr/lib
--sudo ln -s /usr/local/lib/libftd2xx.so.0.4.16 libftd2xx.so.0
--Change your current directory where 99-ftdi.rules is located
--sudo cp 99-ftdi.rules /etc/udev/rules.d
--sudo /etc/init.d/udev restart


JOY NODE:
--gotta figure this one out..


********************************COMPILING***********************************

--catkin_make should compile everything
--there is a chance that you will see an error that arm_init.h cannot be found
--this is apparently an error in ROS (please prove me wrong!)
--fix it by navigating to the build folder and running: make -j4 arm_executive_node4 (or whichever node is giving an issue)


**********************************TO RUN************************************
To initialize the arm:

	roslaunch boost_arm cylinder_arm4.launch


To make the arm configure to the left or the right and then search for a stalk:

	rosservice call arm_init left
	OR
	rosservice call arm_init right


To make the arm go into stow mode:

	rosservice call arm_init stow


To actuate just the gripper:

	rosservice call gripper_close grip
	OR
	rosservice call gripper_open grip


To position the arm joints manually:

	rosrun rosrun hebi armCommandLine Pos_21 Pos_22 Pos_23 Vel_21 Vel_22 Vel_23
	for example: rosrun rosrun hebi armCommandLine NAN 2 NAN 0 0 0

To idle the arm joints:

	rosrun hebi armNAN (I recently saw an issue where the arm_init left command didn't work after calling armNAN. If this happens again, restart boost_arm cylinder_arm4.launch)

To just configure the arm in left/right/stow:
	rosrun hebi arm_config_service_node4
	rosservice call arm_configure left
	




GENERAL OVERVIEW:

The main packages run are:
	- arm_executive arm_executive_node4: this package starts dynamic exposure of the camera, configures the arm to left/right/stow, searches for a stalk, and actuates the gripper
	-  
