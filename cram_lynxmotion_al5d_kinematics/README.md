# The CRAM package to control the Lynxmotion AL5D simulator in Gazebo

This package presents an interface based on designators aimed at controlling the [Lynxmotion AL5D robot simulator](https://github.com/CRAM-Team/lynxmotion_al5d_description/) to perform various pick and place exercises. It also interfaces with the camera sensor attached to the robot. 

The following sections will explain the different capabilities of the package along with examples of how to use them.

## Table of contents

1. [Technical reference documentation](#technical-reference-documentation)
    1. [Motion designators](#motion-designators)
    2. [Process modules](#process-modules)
    3. [Action designators](#action-designators)
    4. [Available fluents](#available-fluents)
2. [Tutorials](#tutorials)

### Technical reference documentation
This section explains the various designators, process modules and fluents available for use in this package. It is intended for developers willing to extend the functionalities for various exercises including but not limited to picking and placing bricks with the robot simulator in Gazebo.

#### Motion designators
For all the designators under this section, the structure used to explain them will be comprised of a name for the motion followed by *the description in italics* and a brief description beneath it. All the values are specified either in metres or radians respectively for distances and angles.

+ Moving - *(desig:a motion (type moving) (position (\<x\> \<y\> \<z\>)) (pitch \<pitch\>) (yaw \<yaw\>))*

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This motion designator gets resolved to an action which sends the robot simulator's T5 frame to the desired position. That means that the specified position is with respect to the wrist axes. The pitch and yaw represent the pose of the object to grasp and represent respectively the rotations around the x and y axis.

+ Grasping - *(desig:a motion (type grasping) (distance \<d\>))*

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This motion designator gets resolved to a function which causes the robot simulator to open its gripper with the specified distance.


#### Process modules

A single process module has been integrated to this package and handles all the motions developed as part of the package. It allows for an automatic resolution of designators and calls the necessary function to process the user's request. The name of the process module is ***lynxmotion_al5d_navigation***.


#### Action designators

For all the action designators under this section, the structure used to explain them will be comprised of a name for the action followed by *the description in italics* and a brief description beneath it. All the values are specified either in metres or radians respectively for distances and angles.

+ Picking - *(desig:an action (type picking) (position (\<x\> \<y\> \<z\>)) (orientation \<angle\>)*

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This action represents a set of motions performed in order to pick a brick a the specified position and orientation. It starts by opening the gripper, going to an approach pose, picking the brick and then return to an approach pose.

+ Approaching - *(desig:an action (type approaching) (position (\<x\> \<y\> \<z\>)) (orientation \<angle\>)*

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This action represents the approach position which takes the wrist (i.e embedding of the T5 frame) at a 0.1 metres distance away from the specified position.

+ Placing - *(desig:an action (type picking) (position (\<x\> \<y\> \<z\>)) (orientation \<angle\>)*

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This action represents the set of motions and actions to complete in order to place a block to a certain position and orientation. At the end of the action, the robot simulator stays at the approach pose.

+ Picking and placing - *(desig:an action (type picking-and-placing) (from (\<src-x\> \<src-y\> \<src-z\>)) (is-heading \<src-orientation\>) (to (\<dst-x\> \<dst-y\> \<dst-z\>)) (will-head \<dst-orientation\>)*

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This represents a full pick and place exercise with the parameters used to specified the start and end pose of a brick to be used.


#### Available fluents

+ \**robot_external_camera_msg*\*

This is a fluent represent the image captured from the camera sensor attached to the robot simulator. A function named *(get-last-captured-image)* allows to retrieve the image as a matrix for various processing tasks.

+ \**robot_joint_states_msg*\*

This fluent represents the current state of all the joints connected to the robot.


### Tutorials
This section will present a set of usage of the package for demonstration purpose. It doesn't explicitly specify how to spawn the bricks that here is assumed to be known by the user. However, independently of that, the below code should work as described.

In order to run the exercice, please open a terminal and execute `roslisp_repl` to start a new LISP session with ROS. You can then type the below function calls at the interpreter prompt.

```lisp
(ros-load:load-system "cram_lynxmotion_al5d_kinematics" :cram-lynxmotion-al5d-kinematics) ; Load the package

(in-package :al5d)

(roslisp-utilities:startup-ros) ;This is a very important step which starts a node and creates the necessary publisher and subscribers to run the program. 

(defparameter *grasping-motion* (desig:a motion (type grasping) (distance 0.03))) ; Defines a motion designator for opening the gripper

(perform-some-motion *grasping-motion*) ;Resolves a process modules and execute the necessary program

(defparameter *pick-and-place-exercice* (desig:an action (type picking-and-placing) (from (0 0.200 0)) (is-heading 0) (to (0.100 0.100 0)) (will-head 3.14159))) ; Defines an action designator to run a pick and place exercice

(top-level
    (with-process-modules-running (lynxmotion-al5d-navigation)
        (exe:perform *pick-and-place-exercice*))) ; Executes the action
```
