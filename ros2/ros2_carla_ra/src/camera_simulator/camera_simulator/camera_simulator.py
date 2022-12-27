
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from sensor_msgs.msg import Image 
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
        self.WIDTH=getParameter(self,"WIDTH")
        self.HEIGHT=getParameter(self,"HEIGHT")
        self.FOV=getParameter(self,"FOV")

        self.connectionTimeoutS=getParameter(self,"connectionTimeoutS")

        self.cameraFirst_tx=getParameter(self,"cameraFirst_tx")
        self.cameraFirst_ty=getParameter(self,"cameraFirst_ty")
        self.cameraFirst_tz=getParameter(self,"cameraFirst_tz")
        self.cameraFirst_rx=getParameter(self,"cameraFirst_rx")
        self.cameraFirst_ry=getParameter(self,"cameraFirst_ry")
        self.cameraFirst_rz=getParameter(self,"cameraFirst_rz")
        self.disableRendering=getParameter(self,"disableRendering")
        self.simulationTimeInS=getParameter(self,"simulationTimeInS")


        self.myPrint("HOST="+str(self.HOST))
        self.myPrint("PORT="+str(self.PORT))
        self.myPrint("WIDTH="+str(self.WIDTH))
        self.myPrint("HEIGHT="+str(self.HEIGHT))
        self.myPrint("FOV="+str(self.FOV))
        self.myPrint("cameraFirst_tx="+str(self.cameraFirst_tx))
        self.myPrint("cameraFirst_ty="+str(self.cameraFirst_ty))
        self.myPrint("cameraFirst_tz="+str(self.cameraFirst_tz))
        self.myPrint("cameraFirst_rx="+str(self.cameraFirst_rx))
        self.myPrint("cameraFirst_ry="+str(self.cameraFirst_ry))
        self.myPrint("cameraFirst_rz="+str(self.cameraFirst_rz))
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

        camera_bp = self.world.get_blueprint_library().filter("sensor.camera.rgb")[0]
        camera_bp.set_attribute("image_size_x", str(self.WIDTH))
        camera_bp.set_attribute("image_size_y", str(self.HEIGHT))
        camera_bp.set_attribute("fov", str(self.FOV))
        
        #tx,ty,tz,rx,ry,rz
        self.poseCamera=[self.cameraFirst_tx,self.cameraFirst_ty,self.cameraFirst_tz,self.cameraFirst_rx,self.cameraFirst_ry,self.cameraFirst_rz]
        #print("!!!configure blueprint done")
        self.cameraFront = self.world.spawn_actor( blueprint=camera_bp, transform=carla.Transform (carla.Location(x=self.poseCamera[0],y=-self.poseCamera[1],z=self.poseCamera[2]),carla.Rotation( roll=np.degrees(self.poseCamera[3]), pitch=np.degrees(self.poseCamera[4]), yaw=-np.degrees(self.poseCamera[5]))) )#, attach_to=self.world.player)
        #print("!!!create self.cameraFront done")			

        """
        pygame.init()
		pygame.font.init()
        self.display = pygame.display.set_mode(
					(self.WIDTH, self.HEIGHT),
					pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.display.fill((0,0,0))
        pygame.display.flip()
        """
        """
        pygame.init()
        self.gameDisplay = pygame.display.set_mode((self.WIDTH,self.HEIGHT))
        pygame.display.set_caption('camera simulator')
        self.gameDisplay.fill((0,0,0))
        pygame.display.flip()
        """

        self.pubCamera                 = self.create_publisher(Image, getCurrentFileName()+"_"+'cameraRGB'  , 10)
        self.pubCameraPosition         = self.create_publisher(Twist, getCurrentFileName()+"_"+'cameraPose'  , 10)
        self.bridge = CvBridge()

        self._timeStamp=-1
        
        #after spwaning, create a call back to get the images
        self.image_queue = Queue()
        #self.cameraFront.listen(lambda data: sensor_callback(data, self.image_queue))
        self.cameraFront.listen(lambda data: self.sensor_callback(data, self.image_queue))
        
    def on_world_tick(self, timestamp):
        
        if self.worldTimeStamp != -1:
            timeEllapsed=timestamp.elapsed_seconds-self.worldTimeStamp
            self.myPrint("timeEllapsed in the work="+str(timeEllapsed))
        self.worldTimeStamp=timestamp.elapsed_seconds
        

    def sensor_callback(self,data, queue):
        #self.myPrint("sensor_callback done#########################")
        if len(data)>0:
            im_array = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype("uint8")))
            im_array = np.reshape(im_array, (data.height, data.width, 4))
            im_array = im_array[:, :, :3][:, :, ::-1]
            #self.myPrint ("img_array.shape= "+str(im_array.shape))


            #self.myPrint ("try display image")
            
            #self.surface = pygame.surfarray.make_surface(im_array)
            #self.gameDisplay.blit(self.surface, (0, 0))

            #self.gameDisplay.fill(im_array)
            #self.myPrint ("try update")
            #pygame.display.update()

            #plt.ion()
            #plt.imshow(im_array)

            """self.myPrint ("try display image")
            cv2.imshow("camera", im_array)
            self.myPrint ("display image")
            cv2.waitKey(1)
            self.myPrint ("display image done")
            """

            self.pubCamera.publish( self.bridge.cv2_to_imgmsg(im_array, "bgr8") )

            #self.pubCamera.publish(im_array)

        #print ("sensor_callback done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        #queue.put(data)


    def __del__(self):
        
        try :
            self.cameraFront.destroy()
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
            #self.myPrint("deltaTimeS  ="+str(deltaTimeS ))

            #(utmx,utmy, tileid, tileLetter) =utm.from_latlon(self.pose[0] ,self.pose[1])
            utmx=self.poseCamera[0]
            utmy=self.poseCamera[1]
            yaw=self.poseCamera[5] #math.pi/2.0-self.pose[5]

            deltaAngle=twist.angular.z*deltaTimeS
            deltaDist=twist.linear.x*deltaTimeS
            
            #self.myPrint("deltaDist  ="+str(deltaDist ))
            #self.myPrint("deltaAngle  ="+str(deltaAngle ))
            
            

            utmx=utmx + deltaDist*math.cos( yaw+deltaAngle/2.0 ) 
            utmy=utmy + deltaDist*math.sin( yaw+deltaAngle/2.0 ) 
            newYaw=yaw+deltaAngle
            
            #lati,longi=utm.to_latlon(utmx,utmy,tileid,tileLetter)
            
            #self.poseCamera = [lati,longi,0.0,0.0,0.0,math.pi/2.0-newYaw]
            self.poseCamera[0]=utmx
            self.poseCamera[1]=utmy
            self.poseCamera[5]=newYaw

            #self.myPrint("    my pose="+str(self.poseCamera))
            #move the camera (carla x formward, y right, so invert y)
            twist=Twist()
            twist.linear.x=self.poseCamera[0]
            twist.linear.y=self.poseCamera[1]
            twist.linear.z=self.poseCamera[2]

            twist.angular.x=self.poseCamera[3]
            twist.angular.y=self.poseCamera[4]
            twist.angular.z=self.poseCamera[5]

            self.pubCameraPosition.publish(twist)
            self.cameraFront.set_transform( carla.Transform (carla.Location(x=self.poseCamera[0],y=-self.poseCamera[1],z=self.poseCamera[2]),carla.Rotation( roll=np.degrees(self.poseCamera[3]), pitch=np.degrees(self.poseCamera[4]), yaw=-np.degrees(self.poseCamera[5])))  )

        self._timeStamp= timeStamp

    def callBackTwistPoses(self,twist):
        self.myPrint("callBackTwist\n")
        self.myPrint("    twist.linear.x="+str(twist.linear.x)+"\n")
        self.myPrint("    twist.linear.y="+str(twist.linear.y)+"\n")
        self.myPrint("    twist.linear.z="+str(twist.linear.z)+"\n")
        self.myPrint("    twist.angular.x="+str(twist.angular.x)+"\n")
        self.myPrint("    twist.angular.y="+str(twist.angular.y)+"\n")
        self.myPrint("    twist.angular.z="+str(twist.angular.z)+"\n")

        self.poseCamera=[twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z]
        #move the camera (carla x formward, y right, so invert y)
        self.cameraFront.set_transform( carla.Transform (carla.Location(x=self.poseCamera[0],y=-self.poseCamera[1],z=self.poseCamera[2]),carla.Rotation( roll=self.poseCamera[3], pitch=self.poseCamera[4], yaw=self.poseCamera[5]))  )
        """
        self.utmx= twist.linear.x
        self.utmy= twist.linear.y
        self.altitude= twist.linear.z
        self.roll= twist.angular.x
        self.pitch= twist.angular.y
        self.yaw= twist.angular.z

        self.sendPose()
        """
        pass

def main(args=None):
    print('Hi from camera_simulator.')
    rclpy.init(args=args)
   
    camera_simulator = CarlaInterface("camera_simulator",rclpy)

    camera_simulator.create_subscription(Twist, getCurrentFileName()+"_"+"poseCameraTwist",camera_simulator.callBackTwistPoses,10)
    camera_simulator.create_subscription(Twist, getCurrentFileName()+"_"+"cmd_vel",camera_simulator.callBackTwistSpeeds,10)

    #node.create_subscription(NavSatFix, getCurrentFileName()+"_"+"navSatFix",robotSender.callBackNavSatFix,10)
    #node.create_subscription(Imu, getCurrentFileName()+"_"+"imu",robotSender.callBackImu,10)
    #node.create_subscription(Twist, getCurrentFileName()+"_"+"poseInUtmTiles",robotSender.callBackTwist,10)

    #while rclpy.ok():
    rclpy.spin(camera_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_simulator.destroy_node()
    rclpy.shutdown()

    return

if __name__ == '__main__':
    main()
