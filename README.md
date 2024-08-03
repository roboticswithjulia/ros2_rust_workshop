1. Build the ros2_rust_dev image

`docker build -f 'src/ros2_rust_workshop/Dockerfile' --build-arg 'ROS_DISTRO=humble' -t ros2_rust_dev .`


2. Run the docker image
   
`docker run -it  ros2_rust_dev`
