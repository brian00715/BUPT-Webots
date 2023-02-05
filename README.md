# Introduction

This project is developed based on the [webots_ros](https://github.com/cyberbotics/webots_ros) offical repo with some necessary customizations for the BUPT robot team's chassis and visual algorithms simulation.

# Features:star2:

- [x] Robocon 2023 field with 1:1 size and collision property.

- [x] Robocon 2022 R1 robot of BUPT (steer wheel chassis with a 2 DoF gripper) in [PROTO](https://www.cyberbotics.com/doc/reference/proto#!) format. The [URDF](http://wiki.ros.org/urdf) file for robot are also completed for further usage (rviz visualization, kinematics, etc.).

- [x] Localization based on GPS and Compass.

# Getting started:rocket:

- Install Webots R2023a
  
  > For better compatibility, this project is developed based on **Ubuntu 20.04** and **ROS Noetic**. We assume that you have already installed ROS on your Linux computer.
  
  Strongly recommend installing Webots using tarball package, refer to [this doc](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-tarball-package).

- Clone and build this repo
  
  > If you haven't installed catkin-tools, install it first:
  > 
  > ```shell
  > sudo apt install python3-catkin-tools
  > ```
  
  ```shell
  # cd to your <ros_workspace>/src and then
  git clone https://gitee.com/SimonKenneth/bupt_webots.git
  catkin build bupt_webots
  source ../devel/setup.bash
  ```

- launch the simulator
  
  ```shell
  roslaunch bupt_webots robocon2023.launch
  ```
  
  Then you will see field and robot are loaded correctly if god bless you 😂.
  
  ![](README_assets/60d385a63977e4eb334a6686fab65075c7530a7c.png)

- Control the motors in Webots
  
  - Using ROS services
    
    For example:
    
    ```shell
    rosservice call /r1/wheel1_drive_joint/set_velocity  "value: 1.0"
    ```
    
    You can create corresponding service clients in your cpp or python nodes which are usually related to kinematics resolving.
  
  - Using `ros_control`
    
    This project has completed [ros_control](http://wiki.ros.org/ros_control) config for Robocon2022 R1 robot. You can simply publish control commands to corresponding topics.
    
    We use velocity control for drive motors and position control for steer motors, the related topics are:
    
    ```
    wheel_drive_velocity_controller/command
    wheel_steer_position_controller/command
    ```
    
    Python example:
    
    ```python
    import rospy
    from std_msgs.msg import Float64MultiArray
    rospy.init_node("drive_vel_pub_test")
    drive_wheel_vel_pub = rospy.Publisher(
    "/r1/wheel_drive_velocity_controller/command",
    Float64MultiArray,
    queue_size=1)
    cmd_vel = Float64MultiArray([1,1,1,1])
    drive_wheel_vel_pub.publish(cmd_vel)
    ```

- Get the localization infomation
  
  We have already realised `localization_webots` node to publish localization infomation. You can simply use it by adding it to `.launch` file. For example:
  
  ```xml
  <launch>
  ...
      <node pkg="bupt_webots" type="localization_webots" name="localization_webots" output="screen">
          <param name="tf_prefix" value="r1"/>
          <param name="localization_type" value="gps"/>
          <param name="publish_rate" value="50"/>
          <param name="publish_tf" value="true"/>
      </node>
  ...
  </launch>
  ```
  
  The localization info will be published to topic `pose`.
  
  If set `publish_tf` to `true`, this node will also publish tf from `/world`  to`<tf_prefix>/base_link` .

Enjoy!

# Develop:wrench:

We strongly recommend you reading the following materials to learn about webots_ros:

- https://www.cyberbotics.com/doc/guide/using-ros
- https://www.cyberbotics.com/doc/reference/ros-api
- [https://www.cyberbotics.com/doc/reference/proto](https://www.cyberbotics.com/doc/reference/proto)

**TODO**