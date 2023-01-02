# Udacity Sensor Fusion Nanodegree, Project 1 Lidar Obstacle Detection

This is a fort from [Udacity's repository for the project](https://github.com/udacity/SFND_Lidar_Obstacle_Detection), where the code is compelted for presentation of a solution for the first project. For general information, please see [Udacity's repository](https://github.com/udacity/SFND_Lidar_Obstacle_Detection).

<img src="media/Viewer.png" width="700" height="400" />

## Development on Ubuntu 20.04

The project work was done on an Ubuntu 22.04 host, running an Ubuntu 20.04 containter. The code has been tested both on the mentioned container as well as Udacity's virtual environment.

If you are using Ubuntu 20.04, you should be able to execute the following command in a shell and see the results as shown in the above picture.

```sh
sudo apt install build-essential cmake git
sudo apt install libpcl-dev
git clone https://github.com/SaidZahrai/SFND_Lidar_Obstacle_Detection 
cd SFND_Lidar_Obstacle_Detection 
mkdir build && cd build
cmake ..
make
./environment
```

