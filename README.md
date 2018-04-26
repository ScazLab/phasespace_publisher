# PhaseSpace Publisher

Bridges the motion capture system PhaseSpace to ROS by providing a translation from PhaseSpace to Baxter's frame of reference. Publishes points from PhaseSpace markers to RVIZ for visualization.


## Prerequisites
* `ros`: Used ROS Indigo
* `PhaseSpace`: PhaseSpace motion caption system, available [here](phasespace.com). Phasespace has to run on Ubuntu 14.04. Phasespace has several dependencies, which you can download from their site:
    * `libowlsock_src`: To build this, cd into the extracted directory `libowlsock_src` and type "make". Copy the compiled `libowlsock.so` from `libowlsock_src` into your working directory.
    * `OWL SDK`: This is only compatible with Python 2.7. After you extract the tarball, copy the `_OWL.so` and `OWL.py` files into your working directory.
* Python libraries: numpy, rospy


## Hardware
* There needs to be a client computer used to run the software and a server computer for PhaseSpace clients to connect to. This is also the computer that the cameras should be attached to.
* Phasespace uses eight cameras set up around the ceiling of the room. They are connected to each other with ethernet cords, one of which is connected to the server computer.
* The LED markers are what the cameras actually track. After you turn them on, they glow a bright red.


## Installation
Compile by running `catkin build phasespace_publisher`. Launch phasespace_publisher with `roslaunch phasespace_publisher phasespace_publisher.launch`. Then open RVIZ (`rosrun rviz rviz`) to see the markers relative to the robot space.

To use PhaseSpace in another project, include phasespace in the launch file. Marker data is published to `/ps_markers/phasespace_points` in a PhaseSpacePtArray.

You can also explore the PhaseSpace system with the Recap GUI (~/PhaseSpace/Recap/recap on the Baxter computer). Double-click on recap to open the program. In the OWL Config tab on the right, click the arrow next to "localhost" under Server Settings. Click "<find servers>" then select ps_server. Make sure the Mode is Default or 1. Click Connect in the top left corner. The markers and cameras will appear. The origin is marked on the lab floor with blue tape.
The scripts folder in ~/PhaseSpace also has examples for using the PhaseSpace. In order to receive data from the markers, something must be run in "master" mode - so if a script uses "slave" mode, run Recap as well.
