# ROS2 Rust workshop using DevContainer

Install the [DevContainer extension in VSCode](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Click on 'Open in DevContainer' in the popup. Check what is going on in the background by clicking 'Show log'.

![images/reopen_in_container.png](images/reopen_in_container.png)

If you missed the popup, you can go to `View` > `Command Palette` > `Dev Containers: ReOpen in Container`.

Note: It will download a lot of things, it may take a while in a slow connection. It took about 3min in my moderate machine with 100mb internet connection.


You will be dropped in a terminal, or you can open another one pressing '+'. The terminal will be inside of docker, and you'll see you are user `cuser@your-machine`.

Everything you do will be saved in the folder of the project (`your_path/ros2_rust_workshop`) (but owned by the container user). You won't lose progress. If things break, remove from `ros_ws` `build`, `install`, `log`. If it goes really bad, remove what is inside of `src` and start over.

Next time you open the project, you can just click on 'Open in DevContainer' and it will open what you built before. If you have any issues, go to `View` > `Command Palette` > `Dev Containers: Rebuild Container`.


# Install dependencies
```bash
# Reminder: You'll see all this cloned in your host, you won't need to do it again
# if you exit/remove this container
cd /workspaces/ros2_rust_workshop/ros_ws/src
git clone https://github.com/roboticswithjulia/ros2_rust_workshop
git clone https://github.com/ros2-rust/ros2_rust.git
vcs import /workspaces/ros2_rust_workshop/ros_ws/src < ros2_rust/ros2_rust_humble.repos
git clone --recursive https://github.com/roboticswithjulia/champ.git -b ros2
```

# Build

Build the workspace, about 3min30s in my moderate machine.

```bash
cd /workspaces/ros2_rust_workshop/ros_ws/
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.sh
colcon build
```

# Run

```bash
source install/setup.bash
ros2 launch champ_config gazebo.launch.py
```