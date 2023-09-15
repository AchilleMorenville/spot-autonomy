docker build -t ros-dev .
docker run -it --rm --name=ros-dev -v "/dev/shm:/dev/shm" ros-dev