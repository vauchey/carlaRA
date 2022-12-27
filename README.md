# carlaRA


## install ros 2  rolling (ubuntu20.0) from https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html


locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt update
sudo apt upgrade

sudo apt install ros-rolling-desktop
sudo apt install ros-dev-tools



## install carla 0.9.13

# -----------install carla from https://carla.readthedocs.io/en/latest/start_quickstart/
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update 
#lister les carla apt-cache madison carla-simulator (rajoute 17Go)

sudo apt-get install carla-simulator=0.9.13
sudo apt-get install carla-simulator=0.9.11

pip3 install --upgrade pip

cd /opt/carla-simulator/PythonAPI/carla/dist
## install python 3.7
#sudo add-apt-repository ppa:deadsnakes/ppa
#sudo apt-get install python3.7
# atester python3.7-venv
pip3 install carla==0.9.13
pip3 install pygame

cd /opt/carla-simulator
./ImportAssets.sh
./CarlaUE4.sh -RenderOffScreen

apt-get install libomp5
./CarlaUE4.sh


#drivers :
ubuntu-drivers devices

https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-20-04-focal-fossa-linux
sudo ubuntu-drivers autoinstall
## install compilation toolchain (docker)
--------------------------	

gsettings set org.gnome.Vino require-encryption false
docker container ls -a


docker build --build-arg EPIC_USER=vauchey --build-arg EPIC_PASS=ghp_wCmfGwuZzUIZfvJi6tPnnLyfU8vs4SoxLl -t carla-prerequisites -f Prerequisites.Dockerfile . 

docker build -t carla -f Carla.Dockerfile .
docker build -t relaxed_babbage -f Carla.Dockerfile .


# creation d'un asset
https://carla.readthedocs.io/en/latest/tuto_A_add_props/
https://carla.readthedocs.io/en/latest/tuto_A_create_standalone/