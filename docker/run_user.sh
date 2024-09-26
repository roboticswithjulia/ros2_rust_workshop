cuser_id="1000"
container_user="cuser"

echo -e "Starting up ros2_rust container \n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"

docker run -it --privileged \
    --user=${cuser_id}:${cuser_id}\
    --group-add sudo \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --workdir="/home/${container_user}/ros2_rust_workshop/" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --cap-add=sys_nice \
    --name="ros2_rust_dev" \
    ros2_rust_dev:latest