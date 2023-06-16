# hdl-localization-ROS2


도커 파일 생성 
nvidia-docker run --privileged -it \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-e NVIDIA_VISIBLE_DEVICES=all \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--net=host \
--ipc=host \
--shm-size=5gb \
--volume=/data/ROS2_robotnav_taeyong_container:/home/ROS2_robotnav_taeyong_container \
--name=ROS2_robotnav_taeyong_container \
--env="DISPLAY=$DISPLAY" \
tyoung96/ros2-foxy-nvidia:1.1

src 폴더 설치
git clone 

필수 패키지 설치
sudo apt install ros-foxy-pcl-ros
sudo apt-get install libpcap-dev
