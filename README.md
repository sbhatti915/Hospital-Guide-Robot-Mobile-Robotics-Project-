# EECE5550 Final Project: Hospital Guide

Team Members: Sameer Bhatti, Jordan Gittleman, Nathaniel Gordon, Shoghair Manjikian

## Dependencies

Beyond ROS Melodic and Turtlebot3, the project includes the following packages:

- [apriltag_ros](http://wiki.ros.org/apriltag_ros)
- [hector_slam](http://wiki.ros.org/hector_slam)
- [explore_lite](http://wiki.ros.org/explore_lite)
- [rqt_joy](https://github.com/aquahika/rqt_virtual_joystick)
- [ImageMagick](https://imagemagick.org/index.php)

## Installation

Begin by navigating to the `/guide` package, our primary package directory:

`$ cd src/guide/`

Download the necessary apriltag meshes:

    $ git clone https://github.com/sharif1093/apriltag_gazebo_model_generator.git

    $ cd ~/apriltag_gazebo_model_generator/ar_tags/scripts

    $ ./generate_markers_model.py -i ../36h11_sample -s 200 -w 50


Re-build the packages to configure the workspace:

`$ catkin_make`

Add the workspace to your ROS environment:

`$ . ~/guide/devel/setup.bash`

## Running

To generate a map file of the hospital world using the `hector_slam` package:

`$ roslaunch guide world.launch`

To autonomously generate a map of the hospital using the `explore_lite` package:

`$ roslaunch guide autoSlam.launch`

You can bring up the hospital world and robot travelling to requested SOI with the following command:

`$ roslaunch guide nav.launch`

This will also launch an instance of RViz and Gazebo.

In a new terminal, you can send commands directly to the `/target` channel using the following script:

`$ rosrun guide target.py`

When prompted, enter an integer 0-6. You can enter multiple numbers in succession to fill the master robot's destination queue.
