# hdl-localization-ROS2

ROS2 wrapper for hdl-localization package with docker

## 1. Build docker image with Dockerfile  

Before you start hdl-localization with docker, you should install [docker](https://www.docker.com/) and [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) in your PC.  

You could also make docker image directly with provieded Dockerfile.  

Move the terminal path to `/docker` and execute the following command.  

```
cd docker
```
```
docker build -t hdl-localization-ros2:latest .
```

`hdl-localization-ros2:latest` is just example of this docker image, you can replace it with the image name you want.  

After the image is created, you can execute `docker images` command to view the following results from the terminal.  

**output** :  

```
REPOSITORY                  TAG                    IMAGE ID          CREATED             SIZE
hdl-localization-ros2       latest                    812ae31625b3   48 minutes ago    3.36GB
```

## 2. Make HDL-Localization-ROS2 docker container  

When you create a docker container, you need several options to use the GUI and share folders.  

First, you should enter the command below in the local terminal to enable docker to communicate with Xserver on the host.  

```
xhost +local:docker
```

After that, make your own container with the command below.  

```
nvidia-docker run --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=${hdl_localization_repo_root}:/root/workspace/src \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --name=${docker container name} \
           --env="DISPLAY=$DISPLAY" \
           ${docker image} /bin/bash
```   

⚠️ **You should change {hdl_localization_repo_root}, {docker container name}, {docker image} to suit your environment.**  

For example,  
```
nvidia-docker run --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=/home/taeyoung/Desktop/hdl-localization-ROS2:/root/workspace/src \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --name=hdl-localization-ros2 \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           hdl-localization-ros2:latest /bin/bash
```

If you have successfully created the docker container, the terminal output will be similar to the below.  

**output** :  

```
================HDL localization ROS2 Docker Env Ready================
root@taeyoung-cilab:/root/workspace#
```  

## 3. Make custom HDL-Localization-ROS2 launch file

When we created our docker container, we utilized the `--volume` option, so the files we modified locally are directly available inside docker.  

**All you need is `.pcd` file, lidar topic and imu topic!**  






## TODO

- [ ] Support hdl_global_localization ROS2 launch file 
- [ ] Code refactoring  
- [ ] Support ROS2 humble  
- [ ] Suuport NDT CUDA


## Acknowledgement

We based ours on the following packages.    
This repository just provides a detailed guide to using docker to reduce dependencies.  

- [DataspeedInc/hdl_localization](https://github.com/DataspeedInc/hdl_localization/tree/ros2)   
- [DataspeedInc/hdl_global_localization](https://github.com/DataspeedInc/hdl_global_localization/tree/ros2)  
- [DataspeedInc/fast_gicp](https://github.com/DataspeedInc/fast_gicp/tree/ros2)
- [tier4/ndt_omp](https://github.com/tier4/ndt_omp)  


