echo -e "Building ros2_rust_workshop:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--tag ros2_rust_dev:latest .