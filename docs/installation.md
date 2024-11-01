## Dependencies

### Install ROS Noetic (Ubuntu 20.04)

### Create a catkin workspace

Create a catkin workspace (if there is none yet). For example, from your home folder:

	cd
	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

The code has been tested on ROS Noetic. Depending on the ROS distribution you installed, you might have to use `kinetic` or `melodic` instead of `noetic` in the previous command.

### Add packages to the catkin workspace

**Clone** this repository into the `src` folder of your catkin workspace.

	cd catkin_ws/src
	git clone git@github.com:tub-rip/ES-PTAM.git

Install catkin dependencies specified in [dependencies.yaml](/dependencies.yaml):

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < ES-PTAM/dependencies.yaml

Additional ROS tools needed (if not already installed):

	sudo apt-get install ros-noetic-image-geometry
	sudo apt-get install ros-noetic-tf-conversions

## Compiling

**Compile the tracking and mapping package**:

	catkin build dvs_tracking mapper_emvs_stereo
	
After building, at least the first time, remember to run:

	source ~/catkin_ws/devel/setup.bash
