This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

----------------------------------------------------------------------------------------------
### Team Members

* Taehee Cho
* John Kneen
* Lingtao Wang

 Three team members were diveded to perception, planning, and control parts.
 
 
---------------------------------------------------------------------------------------------
### tl_detector and tl_classifier

* The model chosen for transfer learning is faster_rcnn_inception_v2_coco.

* Used tensorflow object detection API to fine tune the model for traffic lights.

* Trained the model using tensorflow 1.7 with Cuda 9.0.

* To freeze the model, used tensorflow 1.4 with Cuda 8.0.

* Tested the code with tensorflow 1.3 with Cuda 8.0. 

* With TF1.7+Cuda9.0, the measured inference time was around 45ms. With TF1.3+Cuda8.0, the measured inference time was around 60ms. 

* The 60ms inference time is about 17Hz and this is faster than 10Hz ros camera data update frequency.

* Due to lack of yellow light data set, the model often fails to detect yellow light but for this project, it wasn't necessary to use yellow light to control vehicle. Therefore, there was no further effort to train the model to recognize yellow light.
 

---------------------------------------------------------------------------------------------


### DBW_Node and Twist Controller

The process of the DBW_Node is: (1) read info from dbw_enabled, expected velocity and expected yaw, and the car's current velocity; (2) using PID controller to generate command (Throttle, Brake, and Steering) to match the expected velocity with the current velocity.

a. speed control:

As the PID controller is setup, in every cycle of the loop, the velocity error and the sample time are sent into the PID controller for getting the expected commend for the next step.

The final values for PID contorller are: P: 0.3, I: 0.0025, D: 0.5. It is based on the refresh rate of 50 Hz.

The critical part in the dbw node is to process the output of PID controller. It is because the working mechanism for the speed-up and slow-down are different.

- If the output of the PID controller is positive, the throttle command is used to increase the speed of the car. In order to smooth the acceleration of the car's speed and reduce the jerk, a smooth function is applied.

- If the output of the PID contoller is negative, the brake command is used, by which parameter the weight of the car is included in the brake output.

- One special case is the slow speed region, as the car's speed slow, in order to stop the car completely, a brake value of 700 is needed.


b. steering control:

The steering control is based on the current velocity, expected velocity, and the expected angular_vel. One finding for the yaw_controller is, the quality of the control is highly depended on the delay of the simulation. On the different platform with different delay, the default yaw_controller's effectiveness is different. For the platform with huge delay, the additional pid controller with less P factor but more D factor could improve the control. But for the platform with moderate delay, the default yaw controller is good enough to drive the car within the lane.

After testing on different platform and on different simulations (highway and parking lot), we eventually decided to keep the yaw controller using the default controller mechanism based on the following equation:

 angular_velocity = current_velocity * angular_velocity / linear_velocity


---------------------------------------------------------------------------------------------


Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
