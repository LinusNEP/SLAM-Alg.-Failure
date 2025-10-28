<p align="center">
  <h2 align="center">Understanding why SLAM algorithms fail in modern indoor environments</h2>
  </p>
  
<p align="center">
  <h3 align="center"><a href="https://arxiv.org/abs/2305.05313">arXiv</a> | <a href="https://link.springer.com/chapter/10.1007/978-3-031-32606-6_22#:~:text=In%20real%2Dworld%20environments%2C%20SLAM,position%20and%20the%20environment's%20structure.">Paper</a> | <a href="https://osf.io/qdxev/">Dataset</a></h3>
  <div align="center"></div>
</p>

<p align="center">
  <img src="https://github.com/LinusNEP/RAAD-2023/blob/main/images/description.png" width="100%" />
</p>

LiDAR-based data acquisition | Visual-based data acquisition |
:-: | :-: |
[![Video](https://github.com/LinusNEP/RAAD-2023/blob/main/images/lidarstudyCenter.gif)](https://youtu.be/WKIeU-evmBM) | [![Video](https://github.com/LinusNEP/RAAD-2023/blob/main/images/visionData.png)](https://youtu.be/pBlj0kQzTBY) |


### Table of Contents

- [Introduction](https://github.com/LinusNEP/RAAD-2023#Introduction)
- [Setup Instruction](https://github.com/LinusNEP/RAAD-2023#Setup-Instruction)
- [Run the example files](https://github.com/LinusNEP/RAAD-2023#Run-the-example-files)
- [Citation](https://github.com/LinusNEP/RAAD-2023#Citation)

### Introduction

This repository contains the resources for the paper "[Understanding Why SLAM Algorithms Fail in Modern Indoor Environments](https://link.springer.com/chapter/10.1007/978-3-031-32606-6_22#:~:text=In%20real%2Dworld%20environments%2C%20SLAM,position%20and%20the%20environment's%20structure.)". This paper investigates the reasons for the failures of simultaneous localisation and mapping (SLAM) algorithms in modern indoor environments and provides insights into improving their performance.

### Setup Instruction

- **Set up ROS environment:**

To run the code and reproduce the experiments conducted in the paper, you need to set up a ROS (Robot Operating System) environment. Follow the official ROS documentation [here](https://wiki.ros.org/) to install ROS on your system. We used the ROS Noetic distribution for all our experiments. Make sure you have the necessary dependencies installed.

- **Create a workspace (optional):**
```
mkdir -p ~/o2s_ws/src
cd ~/o2s_ws/src
cd ..
catkin_make
source devel/setup.bash
```
- **Download datasets:**

The datasets are recorded in ROS bag files for each environment. It includes the LiDAR, RGB-D and IMU information from the sensors. You can download the ROS bag files from [here](https://osf.io/qdxev/). Once downloaded, set up the relevant SLAM algorithms.
Lidar-based: [Hector-SLAM](https://github.com/tu-darmstadt-ros-pkg/hector_slam), [Gmapping](https://github.com/ros-perception/slam_gmapping) and [Karto-SLAM](https://github.com/ros-perception/slam_karto). Visual-based: [RTAB](https://github.com/introlab/rtabmap) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).
To run any of the SLAM algorithms on the downloaded dataset, follow the following steps🥇:
```
roscore
```
Verify that the bags contain all the necessary data, e.g.,
```
rosbag info lidarHospitalEnv.bag
```
For lidar-based SLAM using Hector-SLAM, for example, execute the following
```
rosbag play bagfiles/HospitalEnv/lidarHospitalEnv.bag --clock
roslaunch hector_slam_launch tutorial.launch
rosbag record -O lidarHospitalEnvData.bag -a
```
For visual-based datasets using RTAB for example, execute the following

```
rosbag play /bagfiles/HospitalEnv/visualHospitalEnv.bag  --clock
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"
rosbag record -O visualHospitalEnvData.bag -a

```

- **Extract useful data from the SLAM estimated ROS bags:**

Run the `rosbagPOSEtoCSV.py` or `rosbagTFtoCSV.py` scripts. This script provides functionality to extract the tf, odom, pose, etc from the bags. Once all the relevant topics are extracted and saved in CSV format, follow the following examples to replicate our experiments. 

### Run the example files

- **Requirements:**

To run the example codes provided and generate the figures in the paper, you need to have MATLAB installed on your system. Make sure you have a compatible version of MATLAB (at least version 2020a) and the required toolboxes installed. The example codes utilise MATLAB functionalities for data analysis and visualisation.

- **Run and evaluate:**

The example MATLAB codes run in sections. First, you have to use the 'Run' command to load the parameters into the MATLAB workspace. It will return an interactive dialogue box where you have to select yes to load the data into the workspace. Subsequently, for any of the environments, select the metrics of choice (e.g., show the trajectories, absolute trajectory error (ATE), etc) for which you wish to analyse.

Please refer to the paper for more details on the experiments, methodology, and analysis conducted in this work.


### Citation

If you use this work for any academic task, please cite the [paper](https://link.springer.com/chapter/10.1007/978-3-031-32606-6_22#:~:text=In%20real%2Dworld%20environments%2C%20SLAM,position%20and%20the%20environment's%20structure.).
```bibtex
@InProceedings{10.1007/978-3-031-32606-6_22,
author="Nwankwo, Linus and Rueckert, Elmar",
editor="Petri{\v{c}}, Tadej
and Ude, Ale{\v{s}}
and {\v{Z}}lajpah, Leon",
title="Understanding Why SLAM Algorithms Fail in Modern Indoor Environments",
booktitle="Advances in Service and Industrial Robotics",
year="2023",
publisher="Springer Nature Switzerland",
address="Cham",
pages="186--194"
}
```
