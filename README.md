# Co-operative Human Aware Navigation (CoHAN) Planner

The CoHAN Planner provides a set of packages for the Human-Aware Robot Navigation in various contexts. These packages built over ROS navigation stack, includes humans into both global and local planners to plan a human-aware trajectory for the robot considering several social criteria. Our system also provides multiple modes of planning that shift based on the context or can be set manually by simple changing the parameters.  

The system uses [Human-Aware Timed Elastic Band](https://hal.laas.fr/hal-02922029/file/Ro_Man_2020.pdf) (hateb) local planner for human-aware trajectory planning which is based on [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) ROS package. 
![](https://github.com/sphanit/images/blob/main/cohan.png)


Go [here](https://github.com/sphanit/CoHAN_Planner/tree/master) for ros-melodic instalaltion guide.

# Citation 
If you are using our code for your research, please consider citing at least one of our papers (bibtex below).

- P. T. Singamaneni, A. Favier, and R. Alami, “Human-aware navigation planner for diverse human-robot contexts,” in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2021.
- P. T. Singamaneni and R. Alami, “Hateb-2: Reactive planning and decision making in human-robot co-navigation,” in International Conference on Robot & Human Interactive Communication, 2020.

# Installation (ros-kinetic)
1. This installation assumes that the [ROS](http://wiki.ros.org/ROS/Installation) is already installed. Otherwise, please install it before continuing to the next steps. Since, CoHAN is built majorly on ros-melodic, it uses tf2 and is **not compatible** with the standard version of [2D navigation stack](http://wiki.ros.org/navigation). For this installation we use a custom version.
2. Install the requirements
	```
	sudo apt install python-pip python-catkin-tools
	pip install scipy
	```
	If ```pip install scipy``` doesn't work, do ```sudo apt install python-scipy```
3. Clone the git repositories (CoHAN and navigation)
	```
	mkdir -p cohan_ws/src
	cd 	cohan_ws/src
	git clone --recurse-submodules https://github.com/sphanit/CoHAN_Planner.git -b docking-kinetic
	git clone https://github.com/sphanit/navigation.git -b cohan_kinetic
	cd ..	
	```
4. Install the dependencies using rosdep
	```
	rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
	```
5. Build
	```
	catkin build
	```
	Note: catkin build might make the cpu busy utilizing all threads if not configured properly, you can instead use it with less jobs. For example, ```catkin build -j4```, can run up to four jobs at a time.
	
# Usage
1. Our packages are built as plugins to standard 2D navigation stack and hence they follow the same topics to publish navigation goals.

2. To include humans into the system, the tracked humans or the known humans have to be published on ``` /tracked_humans ``` topic following the message structure provided [here](https://github.com/sphanit/CoHAN_Planner/blob/master/human_msgs/msg/TrackedHumans.msg). All the message structures can be found in the ``` human_msgs``` package. The default segement for navigation is TORSO and hence care has to be taken while setting the [type](https://github.com/sphanit/CoHAN_Planner/blob/master/human_msgs/msg/TrackedSegmentType.msg) of ```TrackedSegment```.  

3. Once the ```/tracked_humans``` topic is available, the system automatically starts planning human aware paths and trajectories. 
4. [CoHAN_Navigation](https://github.com/sphanit/CoHAN_Navigation/tree/kinetic-devel) package provides some configuration files and examples for testing the system. It is strongly suggested to check these before you start experimenting.
5. This branch provides additional feature of docking the robot after the end of goal to reach a precise position. Use dynamic re-configure to enable or disable this and to adjust the parameters.

# Bibtex
CoHAN
```
@inproceedings{singamaneni2021human,
  author = {Singamaneni, Phani Teja and Favier, Anthony and Alami, Rachid},
  title = {Human-Aware Navigation Planner for Diverse Human-Robot Ineraction Contexts},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year = {2021},
}
```
HATEB
```
@inproceedings{singamaneni2020hateb,
  author = {Singamaneni, Phani Teja and Alami, Rachid},
  title = {HATEB-2: Reactive Planning and Decision making in Human-Robot Co-navigation},
  booktitle = {International Conference on Robot \& Human Interactive Communication},
  year = {2020},
  doi={10.1109/RO-MAN47096.2020.9223463}}
}
```
