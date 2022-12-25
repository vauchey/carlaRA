
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


import logging




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

    def myPrint(self,msg):
        if self.DEBUG:
            self.get_logger().info(msg)


    def callBackTwist(self,twist):
        self.myPrint("callBackTwist\n)
        self.myPrint("    twist.linear.x="+str(twist.linear.x)+"\n")
        self.myPrint("    twist.linear.y="+str(twist.linear.y)+"\n")
        self.myPrint("    twist.linear.z="+str(twist.linear.z)+"\n")
        self.myPrint("    twist.angular.x="+str(twist.angular.x)+"\n")
        self.myPrint("    twist.angular.y="+str(twist.angular.y)+"\n")
        self.myPrint("    twist.angular.z="+str(twist.angular.z)+"\n")
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

    camera_simulatorcreate_subscription(Twist, getCurrentFileName()+"_"+"poseCameraTwist",camera_simulator.callBackTwist,10)

    #node.create_subscription(NavSatFix, getCurrentFileName()+"_"+"navSatFix",robotSender.callBackNavSatFix,10)
    #node.create_subscription(Imu, getCurrentFileName()+"_"+"imu",robotSender.callBackImu,10)
    #node.create_subscription(Twist, getCurrentFileName()+"_"+"poseInUtmTiles",robotSender.callBackTwist,10)

    #while rclpy.ok():
    rclpy.spin(camera_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    return

if __name__ == '__main__':
    main()
