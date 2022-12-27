
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


        self.LIDAR_MAX_RANGE_IN_M=getParameter(self,"LIDAR_MAX_RANGE_IN_M")
        self.LIDAR_NB_LAYERS=getParameter(self,"LIDAR_NB_LAYERS")
        self.LIDAR_ROTATION_FREQUENCY_HZ=getParameter(self,"LIDAR_ROTATION_FREQUENCY_HZ")
        self.LIDAR_UPPER_FOV_IN_DEG=getParameter(self,"LIDAR_UPPER_FOV_IN_DEG")
        self.LIDAR_LOWER_FOV_IN_DEG=getParameter(self,"LIDAR_LOWER_FOV_IN_DEG")
        self.noise_stddev =getParameter(self,"noise_stddev")
        self.points_per_second=getParameter(self,"points_per_second")

        self.connectionTimeoutS=getParameter(self,"connectionTimeoutS")

        self.lidarFirst_tx=getParameter(self,"lidarFirst_tx")
        self.lidarFirst_ty=getParameter(self,"lidarFirst_ty")
        self.lidarFirst_tz=getParameter(self,"lidarFirst_tz")
        self.lidarFirst_rx=getParameter(self,"lidarFirst_rx")
        self.lidarFirst_ry=getParameter(self,"lidarFirst_ry")
        self.lidarFirst_rz=getParameter(self,"lidarFirst_rz")
        self.disableRendering=getParameter(self,"disableRendering")
        self.simulationTimeInS=getParameter(self,"simulationTimeInS")


        self.myPrint("HOST="+str(self.HOST))
        self.myPrint("PORT="+str(self.PORT))

        self.myPrint("lidarFirst_tx="+str(self.lidarFirst_tx))
        self.myPrint("lidarFirst_ty="+str(self.lidarFirst_ty))
        self.myPrint("lidarFirst_tz="+str(self.lidarFirst_tz))
        self.myPrint("lidarFirst_rx="+str(self.lidarFirst_rx))
        self.myPrint("lidarFirst_ry="+str(self.lidarFirst_ry))
        self.myPrint("lidarFirst_rz="+str(self.lidarFirst_rz))
        self.myPrint("disableRendering="+str(self.disableRendering))
        self.myPrint("simulationTimeInS="+str(self.simulationTimeInS))

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

        
        
       
       	

        

        self.pubLidar                = self.create_publisher(PointCloud2 , getCurrentFileName()+"_"+'lidarXYZ'  , 10)
        self.pubLidarPosition         = self.create_publisher(Twist, getCurrentFileName()+"_"+'lidarPose'  , 10)
        

        self._timeStamp=-1
        
        #after spwaning, create a call back to get the images
        self.lidar_queue = Queue()
        #self.cameraFront.listen(lambda data: sensor_callback(data, self.image_queue))
        self.lidarFront.listen(lambda data: self.sensor_callback(data, self.lidar_queue))
        
    def on_world_tick(self, timestamp):
        
        if self.worldTimeStamp != -1:
            timeEllapsed=timestamp.elapsed_seconds-self.worldTimeStamp
            self.myPrint("timeEllapsed in the work="+str(timeEllapsed))
        self.worldTimeStamp=timestamp.elapsed_seconds
        

    def point_cloud(self,points, parent_frame):
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header(frame_id=parent_frame)

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )


    def sensor_callback(self,data, queue):
        #self.myPrint("lidar sensor_callback done#########################")
        if len(data)>0:
            p_cloud_size = len(data)
            p_cloud = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype('f4')))
            points = np.reshape(p_cloud, (int(p_cloud.shape[0] / 4), 4))#x y z intensity
            #mise dans les axes main droite des donnes lidar
            p_cloud = np.zeros((points.shape[0],3))
            p_cloud[:,0]= points[:,0]#y
            p_cloud[:,1]= -points[:,1]#x
            p_cloud[:,2]= points[:,2]#-z
                    
            p_cloud[:,0]=p_cloud[:,0]
            p_cloud[:,1]=p_cloud[:,1]
            p_cloud[:,2]=p_cloud[:,2]
            
            #p_cloud=p_cloud.reshape(-1).astype("float64")
            #publish p_cloud
            pcd=self.point_cloud(p_cloud,"map")

            self.pubLidar.publish(pcd)
            pass
            """
            im_array = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype("uint8")))
            im_array = np.reshape(im_array, (data.height, data.width, 4))
            im_array = im_array[:, :, :3][:, :, ::-1]
            #self.myPrint ("img_array.shape= "+str(im_array.shape))


            self.pubCamera.publish( self.bridge.cv2_to_imgmsg(im_array, "bgr8") )

            #self.pubCamera.publish(im_array)
            """
        #print ("sensor_callback done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        #queue.put(data)


    def __del__(self):
        
        try :
            self.lidarFront.destroy()
        except:
            print ("self.cameraFront cannot be destroy")
        

    def myPrint(self,msg):
        if self.DEBUG:
            self.get_logger().info(msg)


    def callBackTwistSpeeds(self,twist):
        self.myPrint("callBackTwistSpeeds\n")
        self.myPrint("    twist.linear.x="+str(twist.linear.x)+"\n")
        self.myPrint("    twist.linear.y="+str(twist.linear.y)+"\n")
        self.myPrint("    twist.linear.z="+str(twist.linear.z)+"\n")
        self.myPrint("    twist.angular.x="+str(twist.angular.x)+"\n")
        self.myPrint("    twist.angular.y="+str(twist.angular.y)+"\n")
        self.myPrint("    twist.angular.z="+str(twist.angular.z)+"\n")
        twist.angular.z = twist.angular.z/10.0#slow down the rotations

        timeStamp=(Clock().now().nanoseconds)#time us usrospy.Time.now()
        self.myPrint("timeStamp ="+str(timeStamp))
       
        if self._timeStamp !=-1:
            
            #deltaTimeS=(timeStamp-self._timeStamp).to_sec()#(float(timeStamp-self._timeStamp))/1000000000.0
            deltaTimeS=(timeStamp-self._timeStamp)/1000000000.0#(float(timeStamp-self._timeStamp))/1000000000.0
            self.myPrint("deltaTimeS  ="+str(deltaTimeS ))

            #(utmx,utmy, tileid, tileLetter) =utm.from_latlon(self.pose[0] ,self.pose[1])
            utmx=self.poseLidar[0]
            utmy=self.poseLidar[1]
            yaw=self.poseLidar[5] #math.pi/2.0-self.pose[5]

            deltaAngle=twist.angular.z*deltaTimeS
            deltaDist=twist.linear.x*deltaTimeS
            
            #self.myPrint("deltaDist  ="+str(deltaDist ))
            #self.myPrint("deltaAngle  ="+str(deltaAngle ))
            
            

            utmx=utmx + deltaDist*math.cos( yaw+deltaAngle/2.0 ) 
            utmy=utmy + deltaDist*math.sin( yaw+deltaAngle/2.0 ) 
            newYaw=yaw+deltaAngle
            
            #lati,longi=utm.to_latlon(utmx,utmy,tileid,tileLetter)
            
            #self.poseCamera = [lati,longi,0.0,0.0,0.0,math.pi/2.0-newYaw]
            self.poseLidar[0]=utmx
            self.poseLidar[1]=utmy
            self.poseLidar[5]=newYaw

            self.myPrint("    my pose="+str(self.poseLidar))
            #move the camera (carla x formward, y right, so invert y)
            twist=Twist()
            twist.linear.x=self.poseLidar[0]
            twist.linear.y=self.poseLidar[1]
            twist.linear.z=self.poseLidar[2]

            twist.angular.x=self.poseLidar[3]
            twist.angular.y=self.poseLidar[4]
            twist.angular.z=self.poseLidar[5]

            self.pubLidarPosition.publish(twist)
            self.lidarFront.set_transform( carla.Transform (carla.Location(x=self.poseLidar[0],y=-self.poseLidar[1],z=self.poseLidar[2]),carla.Rotation( roll=np.degrees(self.poseLidar[3]), pitch=np.degrees(self.poseLidar[4]), yaw=-np.degrees(self.poseLidar[5])))  )

        self._timeStamp= timeStamp

    def callBackTwistPoses(self,twist):
        self.myPrint("callBackTwist\n")
        self.myPrint("    twist.linear.x="+str(twist.linear.x)+"\n")
        self.myPrint("    twist.linear.y="+str(twist.linear.y)+"\n")
        self.myPrint("    twist.linear.z="+str(twist.linear.z)+"\n")
        self.myPrint("    twist.angular.x="+str(twist.angular.x)+"\n")
        self.myPrint("    twist.angular.y="+str(twist.angular.y)+"\n")
        self.myPrint("    twist.angular.z="+str(twist.angular.z)+"\n")

        self.poseLidar=[twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z]
        self.pubLidarPosition.publish(twist)
        #move the camera (carla x formward, y right, so invert y)
        self.lidarFront.set_transform( carla.Transform (carla.Location(x=self.poseLidar[0],y=-self.poseLidar[1],z=self.poseLidar[2]),carla.Rotation( roll=np.degrees(self.poseLidar[3]), pitch=np.degrees(self.poseLidar[4]), yaw=-np.degrees(self.poseLidar[5])))  )
      
        pass

def main(args=None):
    print('Hi from lidar_simulator.')
    rclpy.init(args=args)
   
    lidar_simulator = CarlaInterface("lidar_simulator",rclpy)

    lidar_simulator.create_subscription(Twist, getCurrentFileName()+"_"+"poseLidarTwist",lidar_simulator.callBackTwistPoses,10)
    lidar_simulator.create_subscription(Twist, getCurrentFileName()+"_"+"cmd_vel",lidar_simulator.callBackTwistSpeeds,10)

    #node.create_subscription(NavSatFix, getCurrentFileName()+"_"+"navSatFix",robotSender.callBackNavSatFix,10)
    #node.create_subscription(Imu, getCurrentFileName()+"_"+"imu",robotSender.callBackImu,10)
    #node.create_subscription(Twist, getCurrentFileName()+"_"+"poseInUtmTiles",robotSender.callBackTwist,10)

    #while rclpy.ok():
    rclpy.spin(lidar_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_simulator.destroy_node()
    rclpy.shutdown()

    return

if __name__ == '__main__':
    main()
