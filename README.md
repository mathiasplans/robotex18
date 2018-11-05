# robotex18

## [firmware repo](https://os.mbed.com/users/johnsmith312312312/code/ut_bbr_2018/)

## install instructions

### ros setup

Install ROS kinetic for your OS. For windows [this](https://janbernloehr.de/2017/06/10/ros-windows) worked. 

### our core

1. make a folder `mkdir robotex18 && cd robotex18`
2. `git clone --recursive https://github.com/mathiasplans/robotex18.git`
3. rename the inner robotex18 to `src`
4. get build tools
5. get used ros libraries
  * `sudo apt-get install ros-kinetic-cv-bridge`
  * `sudo apt-get install ros-kinetic-vision-opencv`
  * `sudo apt-get install ros-kinetic-image-transport`
6. in robotex18 folder, run make `catkin_make`
7. celebrate
  
