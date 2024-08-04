1. Build the ros2_rust_dev image

`./build_image.sh`


2. Run the docker image
   
`./run_user.sh`


## 2. Quick Start

You don't need a physical robot to run the following demos. If you're building a physical robot, you can find out more how to configure and run a new robot in step 3.

### 2.1 Walking demo in RVIZ:

#### 2.1.1 Run the base driver:

`ros2 launch champ_config bringup.launch.py rviz:=true` 

#### 2.1.2 Run the teleop node:
`docker ps`
`docker exec -it <id> /bin/bash`
`ros2 launch champ_teleop teleop.launch.py` 

If you want to use a [joystick](https://www.logitechg.com/en-hk/products/gamepads/f710-wireless-gamepad.html) add joy:=true as an argument.


### 2.2 Gazebo demo:

#### 2.2.1 Run the Gazebo environment:
    
`ros2 launch champ_config gazebo.launch.py` 

#### 2.2.2 Run [Nav2](https://navigation.ros.org/)'s navigation and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox):
`docker ps`
`docker exec -it <id> /bin/bash`
`ros2 launch champ_config slam.launch.py rviz:=true`

To start mapping:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- Save the map by running:

      cd <your_ws>/src/champ/champ_config/maps
      ros2 run nav2_map_server map_saver_cli -f new_map

After this, you can use the new_map to do pure navigation.

### 2.3 Autonomous Navigation:

#### 2.3.1 Run the Gazebo environment: 

    ros2 launch champ_config gazebo.launch.py

#### 2.3.2 Run [Nav2](https://navigation.ros.org/):

    ros2 launch champ_config navigate.launch.py rviz:=true

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)
