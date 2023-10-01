# Installation
This software depends on [ROS](http://www.ros.org/). Installation instructions can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu). We have tested this software on Ubuntu 20.04. and ROS Noetic.
## Dependencies
Some dependencies need to be installed manually, since **rosdep** omitted them. Below are the links with instructions to install each of them.
- [python3-catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
```
$ sudo sh \
-c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python3-catkin-tools
```
- [vcs-tools](https://github.com/dirk-thomas/vcstool)
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | $ sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python3-vcstool
```
## Setup
Create a ROS workspace and move to the src directory
```
$ mkdir -p ~/emvs_ws/src && cd ~/emvs_ws/src
```
Add the package by cloning the repository into /src
```
$ git clone https://github.com/uzh-rpg/rpg_emvs.git
```
Build the package and source the workspace (always after building)
```
$ catkin build mapper_emvs
$ source ~/emvs_ws/devel/setup.bash
```
# Running Package
## DVS Camera
Initially you can run the rosbag provided in the rpg_emvs repository. (Note: /path/to/... is only a place holder and must be replaced with the correct path to the file).
```
$ roscd mapper_emvs
$ rosrun mapper_emvs run_emvs --bag_filename=/path/to/slider_depth.bag --flagfile=cfg/slider_depth.conf
```
**Note: This ROS package only works with DVS event-camera data, therefore, if another format is used, the data must be reformatted to match DVS.**
If the DVS data does not exist as a ROS bag, a python script "bags.py" is provided to make this conversion. Simply run the python script from the directory of the camera data. Currently, the script works with txt files, however, if the .npz (or similar) are provided, the code can easily be restructured to accommodate this.
```bash
$ python3 bags.py
```
The structure of the .txt files can be found in the [RPG](https://rpg.ifi.uzh.ch/davis_data.html#Rebecq16bmvc) site, to correlate with the python script.
**Note: Be sure to build and source the workspace before running the script, since some it requires the use of the created ROS messages**
### EVIMO Camera
Download the .npz dataset from the [EVIMO](https://better-flow.github.io/evimo/download_evimo_2.html) site. 
The python script "bags_evimo.py" is provided to convert EVIMO camera data into a ROS bag, which can then be used as input for the rpg_emvs package. It is important to note that the structure of the camera data differs from other formats and therefore must be re-arranged utilizing the conversion file. 
Additionally, camera information must be added to the python script, following the notes provided. The structure of the .txt/.npz files can be found on the [EVIMO-Ground-Format](https://better-flow.github.io/evimo/docs/ground-truth-format.html#evimo2v1-vs-evimo2v2) site, to properly modify the necessary field in the python scripts.
```bash
$ python3 bags_evimo.py
```
By default the file should be executed in the same path as the .npz file, but this can be changed by altering the defined paths within the python script.
**Note: Be sure to build and source the workspace before running the script, since some it requires the use of the created ROS messages**
