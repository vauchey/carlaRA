
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
import sensor_msgs
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from sensor_msgs.msg import PointField
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs


#from sensor_msgs.msg import Image 
from sensor_msgs.msg import PointCloud2 
#pip install opencv-python
import cv2
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt

import math
import numpy as np

import logging
#import pygame


import glob
import os   
import sys

CARLA_DIRECTORY="/home/user/carlaRA/carla/CARLA_Shipping_0.9.11-dirty/LinuxNoEditor"
CARLA_DIRECTORY=os.path.abspath(CARLA_DIRECTORY) 
try:
    sys.path.append(glob.glob(CARLA_DIRECTORY+'/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


try:
	import carla
except ImportError:
	raise RuntimeError('cannot import carla, make sure carla package is installed or set CARLA_PYTHON_DIRECTORY')


from queue import Queue
from queue import Empty

def sensor_callback(data, queue):
	
	print ("sensor_callback done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	queue.put(data)


#rostopic echo rosout

"""
if DEBUG :
    logging.basicConfig(level=logging.INFO)
else:
    logging.basicConfig(level=logging.ERROR)
"""

def getCurrentFileName():
    """function to get current filename without slash or extension"""
    return __file__.split("/")[-1].split(".")[0]

def getParameter(node,parameterName):
    """simple function to get easily paramter"""
    fullParameterName=getCurrentFileName()+"_"+str(parameterName)
  
    #node.declare_parameter(fullParameterName,None,dynamic_typing=True)
    #node.declare_parameter(fullParameterName,dynamic_typing=True)
    #node.declare_parameter(fullParameterName,"")
    parameter=ParameterDescriptor()
    parameter.dynamic_typing=True
    node.declare_parameter(fullParameterName,descriptor=parameter)
    parameterValue = node.get_parameter_or(fullParameterName)
    if parameterValue.value is None :
        raise ValueError("parameter "+str(fullParameterName)+" is missing!!")
    #assert( (parameterValue.value is not None), "parameter "+str(fullParameterName)+" is missing!!" )
    return parameterValue.value
   



class CarlaInterface(Node):
    def __init__(self,name,rclpy):
        super().__init__(name)
        self.DEBUG=True

        #myPrint("!!!!!!!!!!!!!!!!!!sys.path="+str(sys.path))
       
        #get parameters
        self.rclpy= rclpy
        self.myPrint("!!init CarlaInterface\n")

        #get configurations
        self.HOST=getParameter(self,"HOST")
        self.PORT=getParameter(self,"PORT")


        self.objectBluePrint=getParameter(self,"objectBluePrint")
       
        self.connectionTimeoutS=getParameter(self,"connectionTimeoutS")

        self.objectFirst_tx=getParameter(self,"objectFirst_tx")
        self.objectFirst_ty=getParameter(self,"objectFirst_ty")
        self.objectFirst_tz=getParameter(self,"objectFirst_tz")
        self.objectFirst_rx=getParameter(self,"objectFirst_rx")
        self.objectFirst_ry=getParameter(self,"objectFirst_ry")
        self.objectFirst_rz=getParameter(self,"objectFirst_rz")
        self.disableRendering=getParameter(self,"disableRendering")
        self.simulationTimeInS=getParameter(self,"simulationTimeInS")


        self.myPrint("HOST="+str(self.HOST))
        self.myPrint("PORT="+str(self.PORT))

      

        self.client = carla.Client(self.HOST, self.PORT)
        self.client.set_timeout(self.connectionTimeoutS)


        #self.world = World(self.client.get_world(), self.hud,carPose, args)
        self.world = self.client.get_world()#get the world

        self.worldTimeStamp=-1
        self.world.on_tick(self.on_world_tick)

        if self.disableRendering:
            self.myPrint("disable rendering")
            settings = self.world.get_settings()
            settings.fixed_delta_seconds = self.simulationTimeInS
            settings.no_rendering_mode = True
            self.world.apply_settings(settings)
        else:
            self.myPrint("enable rendering")
            settings = self.world.get_settings()
            settings.fixed_delta_seconds = self.simulationTimeInS
            settings.no_rendering_mode = False
            self.world.apply_settings(settings)


        """ camera_bp = self.world.get_blueprint_library().filter("sensor.camera.rgb")[0]
        camera_bp.set_attribute("image_size_x", str(self.WIDTH))
        camera_bp.set_attribute("image_size_y", str(self.HEIGHT))
        camera_bp.set_attribute("fov", str(self.FOV))
        """
        #
        myBlueprint = self.world.get_blueprint_library().find(self.objectBluePrint)
        self.poseObject=[self.objectFirst_tx,self.objectFirst_ty,self.objectFirst_tz,self.objectFirst_rx,self.objectFirst_ry,self.objectFirst_rz]
        self.objectFront = self.world.spawn_actor( blueprint=myBlueprint, transform=carla.Transform (carla.Location(x=self.poseObject[0],y=-self.poseObject[1],z=self.poseObject[2]),carla.Rotation( roll=np.degrees(self.poseObject[3]), pitch=np.degrees(self.poseObject[4]), yaw=-np.degrees(self.poseObject[5]))) )#, attach_to=self.world.player)

        """
         #static.prop.chainbarrier  static.prop.turret, , static.prop.palette, static.prop.armoredcar,static.prop.house, static.prop.warehouse static.prop.Warehousevv static.prop.warehousevv
        myBlueprint = blueprint_library.find('static.prop.hangarvv2')#static.prop.box02#static.prop.garbage01 #static.prop.barrier1 static.prop.barrier2
        transform = carla.Transform(carla.Location(x=28.4, y=-4.1, z=500.0), carla.Rotation(yaw=90))
        actor = self.world.spawn_actor(myBlueprint, transform)
        
        blueprint_LID = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        blueprint_LID.set_attribute('range', str(self.LIDAR_MAX_RANGE_IN_M*1000.0))
        blueprint_LID.set_attribute('channels', str(int(self.LIDAR_NB_LAYERS)) )#'16')
        blueprint_LID.set_attribute('points_per_second', str(int(self.points_per_second)))#20hzx100)*16layer
        #blueprint_LID.set_attribute('points_per_second', '56000')#default 56000
        
        blueprint_LID.set_attribute('rotation_frequency', str(int(self.LIDAR_ROTATION_FREQUENCY_HZ)) )#'20')
        
        blueprint_LID.set_attribute('upper_fov', str(self.LIDAR_UPPER_FOV_IN_DEG) )#'15')
        blueprint_LID.set_attribute('lower_fov', str(self.LIDAR_LOWER_FOV_IN_DEG) )#'-15')
        blueprint_LID.set_attribute('noise_stddev', str(self.noise_stddev) ) #'0.02')#add noise

         #tx,ty,tz,rx,ry,rz
        self.poseLidar=[self.lidarFirst_tx,self.lidarFirst_ty,self.lidarFirst_tz,self.lidarFirst_rx,self.lidarFirst_ry,self.lidarFirst_rz]
        #print("!!!configure blueprint done")
        self.lidarFront = self.world.spawn_actor( blueprint=blueprint_LID, transform=carla.Transform (carla.Location(x=self.poseLidar[0],y=-self.poseLidar[1],z=self.poseLidar[2]),carla.Rotation( roll=np.degrees(self.poseLidar[3]), pitch=np.degrees(self.poseLidar[4]), yaw=-np.degrees(self.poseLidar[5]))) )#, attach_to=self.world.player)
        """

       	

        

        self.pubObjectPosition         = self.create_publisher(Twist, getCurrentFileName()+"_"+'objectPose'  , 10)
        

        self._timeStamp=-1
        
        #after spwaning, create a call back to get the images
        #self.lidar_queue = Queue()
        #self.cameraFront.listen(lambda data: sensor_callback(data, self.image_queue))
        #self.lidarFront.listen(lambda data: self.sensor_callback(data, self.lidar_queue))
        
    def on_world_tick(self, timestamp):
        
        if self.worldTimeStamp != -1:
            timeEllapsed=timestamp.elapsed_seconds-self.worldTimeStamp
            self.myPrint("timeEllapsed in the work="+str(timeEllapsed))
        self.worldTimeStamp=timestamp.elapsed_seconds
        

  


    def __del__(self):
        
        try :
            self.objectFront.destroy()
        except:
            print ("self.objectFront cannot be destroy")
        

    def myPrint(self,msg):
        if self.DEBUG:
            self.get_logger().info(msg)


    

    def callBackTwistPoses(self,twist):
        self.myPrint("callBackTwist\n")
        self.myPrint("    twist.linear.x="+str(twist.linear.x)+"\n")
        self.myPrint("    twist.linear.y="+str(twist.linear.y)+"\n")
        self.myPrint("    twist.linear.z="+str(twist.linear.z)+"\n")
        self.myPrint("    twist.angular.x="+str(twist.angular.x)+"\n")
        self.myPrint("    twist.angular.y="+str(twist.angular.y)+"\n")
        self.myPrint("    twist.angular.z="+str(twist.angular.z)+"\n")

        #place the obhect close to the pose sended
        self.poseObject=[twist.linear.x+2.0,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z]
        self.pubObjectPosition.publish(twist)
        #move the camera (carla x formward, y right, so invert y)
        self.objectFront.set_transform( carla.Transform (carla.Location(x=self.poseObject[0],y=-self.poseObject[1],z=self.poseObject[2]),carla.Rotation( roll=np.degrees(self.poseObject[3]), pitch=np.degrees(self.poseObject[4]), yaw=-np.degrees(self.poseObject[5])))  )
      
        pass

def main(args=None):
    print('Hi from object_simulator.')
    rclpy.init(args=args)
   
    object_simulator = CarlaInterface("object_simulator",rclpy)

    object_simulator.create_subscription(Twist, getCurrentFileName()+"_"+"poseObjectTwist",object_simulator.callBackTwistPoses,10)
    #object_simulator.create_subscription(Twist, getCurrentFileName()+"_"+"cmd_vel",object_simulator.callBackTwistSpeeds,10)

    #node.create_subscription(NavSatFix, getCurrentFileName()+"_"+"navSatFix",robotSender.callBackNavSatFix,10)
    #node.create_subscription(Imu, getCurrentFileName()+"_"+"imu",robotSender.callBackImu,10)
    #node.create_subscription(Twist, getCurrentFileName()+"_"+"poseInUtmTiles",robotSender.callBackTwist,10)

    #while rclpy.ok():
    rclpy.spin(object_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_simulator.destroy_node()
    rclpy.shutdown()

    return

if __name__ == '__main__':
    main()
