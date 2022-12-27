FROM carlasim/carla:0.9.11
USER root
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-get update && apt-get install -y xdg-user-dirs xdg-utils && apt-get clean
RUN apt-get install -y sudo

#RUN adduser --disabled-password --gecos '' carla
RUN adduser carla sudo
RUN echo "carla:carla" | chpasswd
USER carla

#docker build -t carla -f test1.Dockerfile .
#to run it

#installs 
#sudo apt install git
#git clone https://github.com/carla-simulator/carla.git

"""
sudo apt-get update
sudo apt upgrade


#on commence par installer les premier spacuqers
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip 
#on rajoute le necessaire pour les autres paquets
sudo apt-get install wget software-properties-common 
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
#enlever libpng16-dev
sudo apt install libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev &&
pip2 install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip2 install --user distro &&
pip3 install --user distro

sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180

git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.24
cd ~/UnrealEngine_4.24

sudo apt install git

# Download and install the UE patches
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/UE4_patch_vulkan.patch
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/UE4_patch_wheels.patch
git apply UE4_patch_vulkan.patch UE4_patch_wheels.patch

# Build UE
./Setup.sh 
./GenerateProjectFiles.sh 
cd ~/UnrealEngine_4.24/Engine/Binaries/Linux && ./UE4Editor
si ca marche pas  au second appel ./UE4Editor ~/carla/Unreal/CarlaUE4/CarlaUE4.uproject

cd
git clone https://github.com/carla-simulator/carla
cd carla 
git checkout 0.9.11
./Update.sh

export UE4_ROOT=~/UnrealEngine_4.24

make PythonAPI
#il y a souvent un probleme de telechargement de pacquet xerces https://github.com/carla-simulator/carla/issues/5846
#nano -w Util/BuilldTools/Setup.sh
#XERCESC_REPO=https://archive.apache.org/dist/xerces/c/3/sources/xerces-c-${XERCESC_VERSION}.tar.gz
make launch

"""
