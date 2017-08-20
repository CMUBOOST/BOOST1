This example was originally created following the example at
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment.
This directory serves as the catkin_ws listed in the tutorial.

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

To make, run:
catkin_make

If this is the first time, you will need to run:
source devel/setup.bash
to set up workspace before running.

To run the application, use:
rosrun <package_directory> <node_name>

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

To create a new package, run:
catkin_create_pkg <packagename> <dependencies>
(see http://wiki.ros.org/catkin/commands/catkin_create_pkg for more details)

You will need to edit the package.xml and CMakeLists.txt files to add
dependencies and names of source files and executables; resources to help you
do this can be found at:
- The bottom of http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
- http://wiki.ros.org/catkin/CMakeLists.txt
- google searches for CMake help :)

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

Send Command:
rostopic pub -1 /joint_commands sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['a', 's', 'd']
position: [0,0, 0]
velocity: [0, 0, 0]
effort: [0, 0, 0]"  

Get Data:
rostopic echo /joint_states

rostopic pub -1 /joint_commands sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['a', 's', 'd']
position: [.4, 0, 0]
velocity: [0, 0, 0]
effort: [0, 0, 0]" 
