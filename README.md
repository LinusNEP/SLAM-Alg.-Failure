<p align="center">

  <h2 align="center">Understanding why SLAM algorithms fail in modern indoor environments</h2>
  This repository contains code and resources related to the paper "Understanding why SLAM algorithms fail in modern indoor environments". The paper investigates the reasons behind the failures of Simultaneous Localization and Mapping (SLAM) algorithms in indoor environments and provides insights into improving their performance.
  <p align="center">
    <a href="https://cps.unileoben.ac.at/m-sc-linus-nwankwo/"><strong>Xingguang Zhong*</strong></a>
    ·
    <a href="https://cps.unileoben.ac.at/prof-elmar-rueckert/"><strong>Yue Pan*</strong></a>
  </p>
  
  <p align="center"><a href="https://www.unileoben.ac.at/"><strong>Montanüniversität Leoben</strong></a>
  
  <h3 align="center"><a href="https://arxiv.org/abs/2305.05313">arXiv</a> | <a href="https://link.springer.com/chapter/10.1007/978-3-031-32606-6_22#:~:text=In%20real%2Dworld%20environments%2C%20SLAM,position%20and%20the%20environment's%20structure.">Paper</a> | <a href="https://osf.io/qdxev/">Dataset</a></h3>
  <div align="center"></div>
</p>

<p align="center">
  <img src="https://github.com/LinusNEP/RAAD-2023/blob/main/images/description.png" width="100%" />
</p>


LiDAR Mapping | Visual Mapping |
:-: | :-: |
<video src='https://github.com/LinusNEP/RAAD-2023/blob/main/images/lidarstudyCenter.mp4'> | <video src='https://youtu.be/pBlj0kQzTBY'> |


<!-- Table of contents -->
<details open="open" style='padding: 10px; border-radius:5px 30px 30px 5px; border-style: solid; border-width: 1px;'>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="##introduction">Introduction</a>
    </li>
    <li>
      <a href="##setup-instruction">Setup instruction</a>
    </li>
    <li>
      <a href="##run-the-example-files">Run the example files</a>
    </li>
    <li>
      <a href="##citation">Citation</a>
    </li>
    <li>
      <a href="##contact">Contact</a>
    </li>
  </ol>
</details>


## Setup Instruction

- **Set up ROS environment:**
To run the code and reproduce the experiments conducted in the paper, you need to set up a ROS (Robot Operating System) environment. Follow the official ROS documentation [here](https://wiki.ros.org/) to install ROS on your system. We used ROS Noetic distribution for all our experiments. Make sure you have the necessary dependencies installed.

- **Create a workspace (optional):**
I called my workspace o2s workspace (o2s_ws). Yours may be different.
```
mkdir -p ~/o2s_ws/src
cd ~/o2s_ws/src
cd ..
catkin_make
```
Set path of the workspace as follows:
```
source devel/setup.bash
gedit ~/.bashrc
```
Add `source ~/o2s_ws/devel/setup.bash` to the very end of the opened bash file so that you don't need to source the bash file each time you open a new terminal window.

- **Download datasets:**
The datasets are recorded in a ROS bag files for each environments. They data include the LiDAR, RGB-D and IMU information from the sensors. In order to analyze the data and reproduce the experiments, you will need to download the rosbag files containing the sensor data from each of the indoor environments. You can download the rosbag files from [here](https://osf.io/qdxev/). Once downloaded, place the rosbag files in the workspace (src) directory.

- **Extract useful data from rosbag file:**
To extract the relevant topics from the rosbag files, you need to run the `rosbagPOSEtoCSV.py` or `rosbagTFtoCSV.py` scripts. This script is included in the repository and provides functionality to extract the tf, odom, pose, etc from the rosbag files. Execute the following command in your terminal:
```
python3 rosbagPOSEtoCSV.py
```
Note, before running the above code, do not forget to replace /home/linus/o2s_ws/src/o2s_robot/bagfile/raadDataset/controlledEnv/lidar/bags/gtCtrldEnv.bag with the actual path to the rosbag file you want to extract topics from. Do same for the output file path (/home/linus/o2s_ws/src/o2s_robot/bagfile/raadDataset/controlledEnv/lidar/gtCtrldEnv.csv). The script will extract the desired topics and save them as csv files for further analysis. Once all the relevant topics are extracted and saved in CSV format, follow the following examples to replicate our experiments. 

## Run the example files

- **Requirements:**
To run the example codes provided in this repository, you need to have Matlab installed on your system. Make sure you have a compatible version of Matlab (at least version 2020a) and the required toolboxes installed. The example codes utilize Matlab functionalities for data analysis and visualization.

- **Clone the GitHub repository:**
To access the example files and scripts used in the paper, clone this GitHub repository to your local PC. Use the following command:
```
cd ~/robot_ws/src
git clone https://github.com/LinusNEP/RAAD-2023.git
```
This will create a local copy of the repository on your PC, allowing you to run the example files and explore the code.

- **Run and evaluate:**
The example matlab codes runs in sections, first you have to use the 'Run' command to load the parameters to matlab workspace, it will return an interactive dialog box where you have to select yes to load the data to the workspace. Subsequently, for any of the environments, select the metrics of choice (e.g., show the trajectories, absolute trajectory error (ATE), etc) for which you wish to analyse.

Please refer to the paper for more details on the experiments, methodology, and analysis conducted in this work.


## Citation
If you use this work for any academic task, please cite our [original paper](https://link.springer.com/chapter/10.1007/978-3-031-32606-6_22#:~:text=In%20real%2Dworld%20environments%2C%20SLAM,position%20and%20the%20environment's%20structure.).
```
@InProceedings{10.1007/978-3-031-32606-6_22,
author="Nwankwo, Linus
and Rueckert, Elmar",
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

## Contact
Should you have any questions, please contact:

- Nwankwo Linus {[linus.nwankwo@unileoben.ac.at]()}


