#! /usr/bin/env python3


#import rospy
#from rospy import Time 

import rclpy
from rclpy.clock import Clock
from rcl_interfaces.msg import ParameterDescriptor


from geometry_msgs.msg import Twist

#pip install scipy
from scipy.spatial.transform import Rotation as R


#geographic_msgs/GeoPose.msg
#from geometry_msgs.msg import GeoPose
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
#je vais publish en mode gms, IMU and Twist utm

#geographic_msgs
#pip install utm
import utm
import math
import numpy as np

DEBUG=False
#rostopic echo rosout

def myPrint(msg):
    if DEBUG:
        #rospy.loginfo(msg)
        print(msg)


def getCurrentFileName():
    """function to get current filename without slash or extension"""
    return __file__.split("/")[-1].split(".")[0]

def getParameter(node,parameterName):
    """simple function to get easily paramter"""
    fullParameterName=getCurrentFileName()+"_"+str(parameterName)
  
    parameter=ParameterDescriptor()
    parameter.dynamic_typing=True
    node.declare_parameter(fullParameterName,descriptor=parameter)
    parameterValue = node.get_parameter_or(fullParameterName)
    if parameterValue.value is None :
        raise ValueError("parameter "+str(fullParameterName)+" is missing!!")
    #assert( (parameterValue.value is not None), "parameter "+str(fullParameterName)+" is missing!!" )
    return parameterValue.value
        

class RobotSimulator:
    def __init__(self,pose,node):
        self.node=node
        self.pose=pose
        self._timeStamp=-1

        #self.pubNavSatFix = rospy.Publisher(getCurrentFileName()+"_"+'navSatFix', NavSatFix, queue_size=10)
        #self.pubImu = rospy.Publisher(getCurrentFileName()+"_"+'imu', Imu, queue_size=10)
        #self.pubTwist = rospy.Publisher(getCurrentFileName()+"_"+'poseInUtmTiles', Twist, queue_size=10)
        self.pubNavSatFix = self.node.create_publisher(NavSatFix, getCurrentFileName()+"_"+'navSatFix'  , 10)
        self.pubImu       = self.node.create_publisher(Imu, getCurrentFileName()+"_"+'imu'  , 10)
        self.pubTwist     = self.node.create_publisher(Twist, getCurrentFileName()+"_"+'poseInUtmTiles'  , 10)
        pass

    def callback(self,twist):
        myPrint ("!!!!!!!!!!callback")
        myPrint("twist.linear.x ="+str(twist.linear.x))
        myPrint("twist.linear.y ="+str(twist.linear.y))
        myPrint("twist.linear.z ="+str(twist.linear.z))
        myPrint("twist.angular.x ="+str(twist.angular.x))
        myPrint("twist.angular.y ="+str(twist.angular.y))
        myPrint("twist.angular.z ="+str(twist.angular.z))

        myPrint("self.pose[0]  ="+str(self.pose[0] ))
        myPrint("self.pose[1]  ="+str(self.pose[1] ))
    
        #rospy.Time.now()
        timeStamp=(Clock().now().nanoseconds)#time us usrospy.Time.now()
        myPrint("timeStamp ="+str(timeStamp))
       
        if self._timeStamp !=-1:
            
            #deltaTimeS=(timeStamp-self._timeStamp).to_sec()#(float(timeStamp-self._timeStamp))/1000000000.0
            deltaTimeS=(timeStamp-self._timeStamp)/1000000000.0#(float(timeStamp-self._timeStamp))/1000000000.0
            myPrint("deltaTimeS  ="+str(deltaTimeS ))

            (utmx,utmy, tileid, tileLetter) =utm.from_latlon(self.pose[0] ,self.pose[1])
            
            yaw=math.pi/2.0-self.pose[5]

            deltaAngle=twist.angular.z*deltaTimeS
            deltaDist=twist.linear.x*deltaTimeS
            
            myPrint("deltaDist  ="+str(deltaDist ))
            myPrint("deltaAngle  ="+str(deltaAngle ))
            
            

            utmx=utmx + deltaDist*math.cos( yaw+deltaAngle/2.0 ) 
            utmy=utmy + deltaDist*math.sin( yaw+deltaAngle/2.0 ) 
            newYaw=yaw+deltaAngle
            
            lati,longi=utm.to_latlon(utmx,utmy,tileid,tileLetter)
            
            self.pose = [lati,longi,0.0,0.0,0.0,math.pi/2.0-newYaw]
            myPrint("self.pose  ="+str(self.pose ))


            navSatFix =NavSatFix()
            imu=Imu()
            twist=Twist()

            navSatFix.latitude=self.pose[0]
            navSatFix.longitude=self.pose[1]
            navSatFix.altitude=self.pose[2]
            quat=R.from_euler("ZYX",[self.pose[5],self.pose[4],self.pose[3]],degrees=False).as_quat()
            #quat=quaternion_from_euler(self.pose[3], self.pose[4], self.pose[5])#r,p,y
            imu.orientation.w=quat[0]
            imu.orientation.x=quat[1]
            imu.orientation.y=quat[2]
            imu.orientation.z=quat[3]

            twist.linear.x=utmx
            twist.linear.y=utmy
            twist.linear.z=self.pose[2]

            twist.angular.x=self.pose[3]
            twist.angular.y=self.pose[4]
            twist.angular.z=self.pose[5]


            self.pubNavSatFix.publish(navSatFix)
            self.pubImu.publish(imu)
            self.pubTwist.publish(twist)

            pass
        self._timeStamp= timeStamp

"""
def callback(twist):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print ("!!!!!!!!!!callback")
    myPrint("twist.linear.x ="+str(twist.linear.x))
    myPrint("twist.linear.y ="+str(twist.linear.y))
    myPrint("twist.linear.z ="+str(twist.linear.z))
    myPrint("twist.angular.x ="+str(twist.angular.x))
    myPrint("twist.angular.y ="+str(twist.angular.y))
    myPrint("twist.angular.z ="+str(twist.angular.z))
    pass
"""

def main(args=None):
    #rospy.init_node('my_broadcaster')
    rclpy.init(args=args)
    node = rclpy.create_node("my_broadcaster")

    print ("!!!!!!subscriber call")

   
    #get parameters
    defaultLatitude=getParameter(node,"defaultLatitude")
    defaultLongitude=getParameter(node,"defaultLongitude")
    defaultYawDeg=getParameter(node,"defaultYawDeg")
    connectionTimeoutS=getParameter(node,"connectionTimeoutS")
    pose=[defaultLatitude, defaultLongitude, 0.0,0.0,0.0,np.radians(defaultYawDeg)]

    robotSimulator =RobotSimulator(pose,node)

    self.client = carla.Client(self.HOST, self.PORT)
	self.client.set_timeout(connectionTimeoutS)
    

    #rospy.Subscriber(getCurrentFileName()+"_"+"cmd_vel", Twist, robotSimulator.callback)
    node.create_subscription(Twist, getCurrentFileName()+"_"+"cmd_vel",robotSimulator.callback,10)
    #rclpy.spin(minimal_subscriber)
    rclpy.spin(node)

if __name__ == '__main__':
    main()