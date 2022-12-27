
# commencer par installer les drivers nvdia  unbuntu 
https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu

 nvidia-smi
 grep "X Driver" /var/log/Xorg.0.log
 
sudo apt install git
 
 ## installer docker nvdia
 
sudo apt-get update
sudo apt-get remove docker docker-engine docker.io
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker
docker --version


 sudo groupadd docker
sudo usermod -a -G docker $USER
newgrp docker


sudo apt install curl
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/


sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   
sudo apt-get update

sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

##si cette ligne fonctionne, on va pouvoir avoir des dockers avec acces au graphique :
docker run --gpus all nvidia/cuda:10.0-base nvidia-smi

# commencer a  creer le docker carlla de lancement (apres on fera la compil dedans, mais d'abord vérifier que tout est ready)
git clone https://github.com/carla-simulator/carla.git
cd carla 
git checkout 0.9.11


docker pull carlasim/carla:0.9.11
docker build -t carlacustom -f test1.Dockerfile .
## lancement du docker en mode graphique avec tout 
docker run --privileged -it --gpus all --net=host -e DISPLAY=$DISPLAY -e SDL_VIDEODRIVER=x11 -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlacustom /bin/bash 
./CarlaUE4.sh -vulkan

## gestion des container 
docker container list -a
docker rm vigilant_chatelet
docker start amazing_aryabhata
docker attach amazing_aryabhata
docker cp ./mgs_barriers.zip amazing_aryabhata:/home/carla/ressources/
docker cp ./scopesPackage amazing_aryabhata:/home/carla/carla/Import/


<!--
#docker run -p 2000-2002:2000-2002 --runtime=nvidia --gpus all carlasim/carla:0.9.11
#sudo docker run   -e SDL_VIDEODRIVER=x11   -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix   -p 2000-2002:2000-2002   -it   --gpus all   carlasim/carla:0.9.11 ./CarlaUE4.sh -opengl
#docker run   -e SDL_VIDEODRIVER=x11   -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix   -p 2000-2002:2000-2002   -it   --gpus all   carlasim/carla:0.9.11 ./CarlaUE4.sh
#docker run   -e SDL_VIDEODRIVER=x11   -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix   -p 2000-2002:2000-2002   -it   --gpus all   carla
#docker run -p 2000-2002:2000-2002 -it --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 carla /bin/bash CarlaUE4.sh
#lancement du docker 

#ca c'est ok pour jouer en mode sans opengl
#docker run   -e SDL_VIDEODRIVER=x11   -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix   -p 2000-2002:2000-2002   -it   --gpus all carla /bin/bash

#ca sa marche, je vais me baser la dessus
#docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY -e SDL_VIDEODRIVER=x11 -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.11 /bin/bash ./CarlaUE4.sh -vulkan
-->

# install ros 2 from https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

apt-cache policy | grep universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt upgrade # attention risque de changer la version de carla

sudo apt install ros-rolling-desktop
sudo apt install ros-rolling-ros-base
sudo apt install ros-dev-tools

** to source ros 2 source /opt/ros/rolling/setup.bash

## creation d'un noeud ros2 python
cd ~/home/user/carlaRA
mkdir -p ros2/ros2_carla_ra/src
cd ros2/ros2_carla_ra/src

source /opt/ros/rolling/setup.bash

ros2 pkg create --build-type ament_python --node-name camera_simulator camera_simulator
ros2 pkg create --build-type ament_python --node-name lidar_simulator lidar_simulator
ros2 pkg create --build-type ament_python --node-name object_simulator object_simulator
ros2 pkg create --build-type ament_python --node-name robot_simulator robot_simulator
pip3 install scipy
pip3 install utm