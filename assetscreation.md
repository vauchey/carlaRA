# creation d'un asset
https://carla.readthedocs.io/en/latest/tuto_A_add_props/
https://carla.readthedocs.io/en/latest/tuto_A_create_standalone/

cd Import
mkdir customObjecct1

## creation d'un fichier json du type :
nano -w customOject1.json

{
  "maps": [
  ],
  "props": [
    {
      "name": "Barrier1",
      "size": "small",
      "source": "./mgs_barriers/barrier1.fbx",
      "tag": "MyBarrier1"
    },
     {
      "name": "Barrier2",
      "size": "small",
      "source": "./mgs_barriers/barrier2.fbx",
      "tag": "MyBarrier2"
    }
  ]
}

<!--
#creation de lasset:
#from https://free3d.com/3d-model/concrete-road-construction-and-armored-military-barriers-532580.html
#~/ressources/mgs_barriers
#mgs_barriers/barrier1.fbx
#mgs_barriers/barrier2.fbx 
-->

## dans le docker, essayer
export UE4_ROOT=~/UnrealEngine_4.24
make import
make package ARGS="--package=customOject1"
make package ARGS="--package=scopesPackage"
make plugins # si probleme pourfaire l package
<!-- make package-->
make package ARGS="--package=customOject1"
#search /home/ue4/carla/Dist/*.tar.gz
#/home/carla/carla/Dist/CARLA_0.9.11-dirty.tar.gz

# install carla ubuntu
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update
sudo apt-get install carla-simulator=0.9.11


# install python and utils
sudo apt install python pip
pip3 install --user pygame numpy

cd /opt/carla-simulator
cd /opt/carla-simulator/Import
sudo docker cp  amazing_aryabhata:/home/carla/carla/Dist/customOject1_0.9.11-dirty.tar.gz ./
sudo docker cp  amazing_aryabhata:/home/carla/carla/Dist/scopesPackage_0.9.11-dirty.tar.gz ./

sudo ./ImportAssets.sh
./CarlaUE4.sh -quality-level=Low

# test d'importation de lieu
https://www.turbosquid.com/3d-models/max-warehouse-pbr-gaming/1017253#
https://products.aspose.app/3d/viewer/fbx
https://3dviewer.net/
https://free3d.com/3d-model/house-11303.html

## install blender pour coller les texutres
sudo snap install blender --classic
/home/user/carlaRA/house/House
/home/user/carlaRA/turret/Sci Fi AA Turret-Fbx
https://www.turbosquid.com/fr/AssetManager/Index.cfm?stgAction=getFiles&subAction=Download&intID=1649704&intType=3&csrf=3F5868DF6425F6E34E8862444B812E688AB1C073&showDownload=1&s=1
visualisateur https://3dviewer.net/


# injection de la 3D
Ouvrir unreal engine, inporter le fbx , puis enregister .
compiler la package 
make package
* extraire et tester l'archive generer dans /home/carla/carla/Dist/CARLA_Shipping_0.9.11-dirty.tar.gz
par exemple :
* docker cp amazing_aryabhata:/home/carla/carla/Dist/scopesPackage_0.9.11-dirty.tar.gz ./scopesPackage_0.9.11-dirty.tar.gz
docker cp amazing_aryabhata:/home/carla/carla/Dist/CARLA_Shipping_0.9.11-dirty CARLA_Shipping_0.9.11-dirty
cd CARLA_Shipping_0.9.11-dirty/LinuxNoEditor
! attention, les objets ne sont pas vus comme de la 3D.
## injection des objets 
cd Import$
docker cp amazing_aryabhata:/home/carla/carla/Dist/scopesPackage_0.9.11-dirty.tar.gz ./scopesPackage_0.9.11-dirty.tar.gz
cd ..
./ImportAssets.sh