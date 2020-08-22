# CRAM packages to control the Lynxmotion AL5D simulator
- [About the package](#about-the-package)
- [Installation](#installation)

## About the package

This represents a set of packages created to control the [robot simulator of the Lynxmotion AL5D robot arm](https://github.com/CRAM-Team/lynxmotion_al5d_description) using the CRAM Programming Language (CPL). It comprises three different packages namely:

+ **cram_lynxmotion_al5d_description**
    + [ ] not working
    > This package is used to describe the robot model to CRAM in order to spawn and control the robot in the Bullet world. At the moment, the package is not working as expected. At the current stage, spawning the robot in the Bullet world shows the robot with the links in various positions as shown in the image below. [#1](https://github.com/CRAM-Team/cram_lynxmotion_al5d/issues/1)
    
    ![Visualization of the robot in the Bullet world ](screenshots/bullet_visualization.png?raw=true)

+ **cram_lynxmotion_al5d_demo**
    + [ ] not working
    > This package bases its functionement depends on the previous one. It is supposed to load a demo environment in which various exercises will be run. At the moment, the development is put on hold.

+ **cram_lynxmotion_al5d_kinematics**
    + [x] working
    > This package makes use of the CPL package to control the Lynxmotion AL5D robot simulator in the Gazebo simulation world. It allows to compute the joint angles necessary to send the robot to a particular position. It also allows to have direct access to the image captured by the external camera sensor attached with the robot. The detailed documentation of the package is available in the [package subdirectory](cram_lynxmotion_al5d_kinematics/)

## Installation
First clone the package to the *cram* subdirectory under the ros workspace by doing the following.
```bash
roscd

cd ../src/cram

git clone https://github.com/CRAM-Team/cram_lynxmotion_al5d.git
```

After that, you will have to build the package by running the following commands:

```bash 
roscd

cd ..

catkin_make
```
After that, you are ready to run the package.
