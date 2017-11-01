#  Bebop_ROS_Examples

This is a compilation of examples ready-to-use for Parrot's Bebop Drone, these examples depends of the following:
  - ROS Kinetic
  - Bebop Autonomy 
  - Vicon Bridge

These are the examples in this repository:
## Simple Test

  - *Package:* **bebop_simple_test**
  - *Node:* **bebop_test**
   
  This program allow to the user to take-off, landing and move across the space using the sensors in the drone. There are four test, and can be selected at the beginning of the source code:

-- [**0**] SIMPLE TAKE-OFF & LANDING: It performs a quick takeoff and
landing at the same place.
-- [**1**] SIMPLE ROUTINE: It performs a actions like elevate, descend,
hover and rotating at certain headings.
-- [**2**] COMPLEX ROUTINE: It performs combined actions (elevate/descend
as it rotates at certain heading).
-- [**3**] CONTINUOUS ROTATING: It keeps a specified altitude and rotates
infinitely until < Ctrl + C > is pressed and lands.

## Datalogger

  - *Package:* **datalogger**
  - *Node:* **datalogger_node**
   
  This tool capture data from the drone sensors and programs and it exports them into a CSV file:

+ Altitude (Ultrasonic Sensor)
+ Odometry
+ PD Outputs 

The output file can be easily used in many Spreadsheet programs and MATLAB.

## Odometry

  - *Package:* **odometry**
  - *Node:* **odometry**

This tool visualizes the different data acquired by Bebop Autonomy.

## OpenCV Example

  - *Package:* **opencv_example**
  - *Node:* **opencv_example**

This program shows the capability to use the image stream from the
drone and prepare it for its use with OpenCV.

## Vicon Test

  - *Package:* **vicon_test**
  - *Node:* **vicon_test_node**
   
  This program performs a basic routine with the vicon camera system using waypoints, which are configurable at the code, the drone always will keep its sight to the center of the world, to avoid any unpredictable behavior this last action will not be effectuated until the drone is outside of a deadzone area, also configurable.

****Note: To stop the program and land the drone push down and hold the Button[0] of your joystick, to know which button is, open a new terminal with the rostopic echo /joy command and press the buttons until it change from a state of 0 to 1.***

## Vicon Wand

  - *Package:* **vicon_wand**
  - *Node:* **vicon_wand_node**

This program need a second object created in the vicon system as a wand, the drone will try to move to the front of the wand and change its heading to match the face of the wand. **Be aware of your surroundings!**

****Note: To stop the program and land the drone push down and hold the Button[0] of your joystick, to know which button is, open a new terminal with the rostopic echo /joy command and press the buttons until it change from a state of 0 to 1.***

## Vicon Mimo Bebop

  - *Package:* **vicon_mimo_bebop**
  - *Node:* **vicon_mimo_bebop_node**
   
This program requires a second Parrot Bebop 1 registered in the vicon system, refer to the configuration of a drone for the vicon programs. This program in particular, does not interact with the joystick, instead, it listen only the messages of take-off and land that are sent to the master drone (the default namespace bebop of the bebop driver), and send it to the slave drone too.

Any executable that uses Bebop Autonomy and a joystick to manually pilot the drone works, like the manual control node of Ric Fehr (visit: https://github.com/ricfehr3/updated_testing_demos/tree/master/bebop_control/src).

Just open a new terminal, follow the creators instructions and execute it.

Its important to give the drones a distance of at least three and a half feet between them using the Y- axis of the world as reference.

As the master drone moves across the space the slave drone will try to replicate its movements using the position and orientation of the first one, keeping a distance between them.
