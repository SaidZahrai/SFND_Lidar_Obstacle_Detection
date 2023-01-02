# Udacity Sensor Fusion Nanodegree, Project 1 Lidar Obstacle Detection

This is a fort from [Udacity's repository for the project](https://github.com/udacity/SFND_Lidar_Obstacle_Detection), where the code is compelted for presentation of a solution for the first project. For general information, please see [Udacity's repository](https://github.com/udacity/SFND_Lidar_Obstacle_Detection).

<img src="media/Viewer.png" width="700" height="400" />

## Running the code

The project work was done on an Ubuntu 22.04 host, running an Ubuntu 20.04 containter. The code has been tested both on the mentioned container as well as Udacity's virtual environment.

### Ubuntu 20.04
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

### Docker container
If you are not using Ubuntu 20.04, a simple way to move forward would be to use a docker container. With some seraches on internet and lots of experimentation, I succeeded to have the container set up for my machine with the `Dockerfile` that you find in the repository.

You should first build the container with
```sh
git clone https://github.com/SaidZahrai/SFND_Lidar_Obstacle_Detection 
cd SFND_Lidar_Obstacle_Detection 
docker build -t sfnd_d .
```
and then start the docker container with the script `start_cmd` in the repository, which contains
```sh
xhost +local:root

docker run -ti --rm  \
       -u 1000:1000 \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix  \
       -v ~/git/SFND_Lidar_Obstacle_Detection:/home/developer/SFND \
       -w /home/developer/SFND \
       --device /dev/dri --group-add=$(stat -c "%g" /dev/dri/render*) \
       sfnd_d /bin/bash
       
xhost -local:root
```
Note that I clone the repository in `~/git/.` and then in the above command, I map that to `/home/developer/SFND/` which will also be the entry point. In that case, once you are on the container you can continue with the following commands
```
mkdir build && cd build
cmake ..
make
./environment
```

The major difficulty I met was to have the graphics workig. With the above setup, I map the drivers so that the virtual machine access the graphics hardware directly. Note that I had Intel graphics hardware. You need to adapt your steps, or use a different base image, in case you have Nvidia, ot anther hardware.