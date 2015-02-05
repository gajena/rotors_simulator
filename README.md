RotorS
===============

RotorS is a UAV gazebo simulator.


Installation Instructions
-------------------------

 1. Install ROS indigo desktop full and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full
 $ sudo apt-get install python-wstool
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone git@github.com:ethz-asl/euroc_simulator.git
 $ git clone git@github.com:ethz-asl/mav_comm.git
 $ git clone git@github.com:ethz-asl/glog_catkin.git
 $ git clone git@github.com:catkin/catkin_simple.git
 $ sudo apt-get install ros-indigo-octomap-ros
 ```
 4. Build your workspace
   - You can either do so by catkin_make

     ```
     $ cd ~/catkin_ws/
     $ catkin_make
     ```
   - Or with `python_catkin_tools` (therefore you need `python_catkin_tools`)
     1. Get the `python_catkin_tools`

       ```
       $ sudo apt-get install python-catkin-tools
       ```
     2. Build with `catkin_tools`

       ```
       $ cd ~/catkin_ws/
       $ catkin init  # If you haven't done this before.
       $ catkin build
       ```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec FireFly.

```
$ roslaunch mav_gazebo firefly_empty_world.launch
```
The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/euroc_simulator/mav_gazebo/launch`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motors mav_msgs/CommandMotorSpeed '{motor_speed: [100, 100, 100, 100, 100, 100]}'
```

> **Note** that the size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch mav_gazebo firefly_hovering_example.launch
```

#### Create an attitude controller

**TODO(ff):** `Write something here.`

#### Usage with a joystick

**TODO(ff):** `Write something here.`
