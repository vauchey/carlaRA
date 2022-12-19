

https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu

 nvidia-smi
 grep "X Driver" /var/log/Xorg.0.log
 
 sudo apt install git
 
 
 sudo apt-get update
sudo apt-get remove docker docker-engine docker.io
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker
docker --version

 perso 
 https://gamcore.com/games/our_red_string
 https://gamcore.com/games/lewd_island
 
 ########"
 sudo groupadd docker
sudo usermod -a -G docker $USER
newgrp docker


sudo apt install curl
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.gith	ub.io/nvidia-docker/$distribution/


sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

docker run --gpus all nvidia/cuda:10.0-base nvidia-smi

########
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   
sudo apt-get update

sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker


#https://github.com/carla-simulator/carla.git
cd carla 
git checkout 0.9.11


docker pull carlasim/carla:0.9.11
docker build -t carlacustom -f test1.Dockerfile .
docker run --privileged -it --gpus all --net=host -e DISPLAY=$DISPLAY -e SDL_VIDEODRIVER=x11 -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlacustom /bin/bash 
#relancement
docker start amazing_aryabhata
docker attach

./CarlaUE4.sh -vulkan


"""
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
"""


