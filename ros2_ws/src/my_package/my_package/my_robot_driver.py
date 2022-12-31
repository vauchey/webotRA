
import rclpy
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

"""
/clock
/my_robot/Velodyne_VLP_16/point_cloud
/my_robot/camera
/my_robot/camera/camera_info
/my_robot/ds0
/my_robot/ds1
/my_robot/lidar/point_cloud
/parameter_events
/remove_urdf_robot
/robot_cmd_pos
/robot_cmd_vel
/rosout
/tf_static


/clock
/joint_states
/my_robot/Velodyne_VLP_16/point_cloud
/my_robot/camera
/my_robot/camera/camera_info
/my_robot/ds0
/my_robot/ds1
/my_robot/lidar/point_cloud
/parameter_events
/remove_urdf_robot
/robot_cmd_pos
/robot_cmd_vel
/robot_description
/rosout
/tf
/tf_static


ros2 topic pub /robot_cmd_vel geometry_msgs/Twist  "angular: { z: 0.1 }"
ros2 topic pub /obj_cmd_vel geometry_msgs/Twist  "angular: { z: 0.1 }"

ros2 topic pub /obj_cmd_pos geometry_msgs/Twist  "linear: { x: 0.2, y: 0.0, z: 0.2 }"

#https://cyberbotics.com/doc/guide/supervisor-programming?tab-language=python
ros2 topic pub /robot_cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
ros2 topic pub /robot_cmd_vel geometry_msgs/Twist  "angular: { z: 0.1 }"
ros2 topic echo /my_robot/Velodyne_VLP_16/point_cloud
ros2 topic echo /my_robot/lidar/point_cloud
 self.__target_twist_pose





ros2 topic pub /obj_cmd_pos geometry_msgs/Twist  "angular: { x: 1.5, y: 0.0, z: 0.0 }"
ros2 topic pub /robot_cmd_pos geometry_msgs/Twist  "linear: { x: 0.0 ,y: 0.0, z: 2.0 }"
ros2 topic pub /robot_cmd_pos geometry_msgs/Twist  "linear: { x: 0.0 ,y: 0.0, z: 2.0 }, angular: { x: 0.0 ,y: 0.0, z: 0.0  }"
ros2 pub -1 /cmd_po geometry_msgs/Twist -- '[0.0, 0.0, 1.0]' '[0.0, 0.0, 1.8]'

from controller import Supervisor
supervisor = Supervisor()
robot_node = supervisor.getFromDef("P1")
trans_field = robot_node.getField("translation")
print ("trans_field="+str(trans_field))
values = trans_field.getSFVec3f()
print("P1 is at position: %g %g %g" % (values[0], values[1], values[2]))

robot_node = supervisor.getFromDef("M1")
trans_field = robot_node.getField("translation")
print ("trans_field="+str(trans_field))
values = trans_field.getSFVec3f()
print("M1 is at position: %g %g %g" % (values[0], values[1], values[2]))


from controller import Supervisor
supervisor = Supervisor()
robot_node = supervisor.getFromDef("P1")
trans_field = robot_node.getField("translation")
print ("trans_field="+str(trans_field))
values = trans_field.getSFVec3f()
print("P1 is at position: %g %g %g" % (values[0], values[1], values[2]))

robot_node = supervisor.getFromDef("M1")
print ("dir(robot_node)="+str(dir(robot_node)))
trans_field = robot_node.getField("translation")
rotation_field=robot_node.getField("rotation")
print ("trans_field="+str(trans_field))
print ("rotation_field="+str(rotation_field))
print ("dir(rotation_field)="+str(dir(rotation_field)))
values = trans_field.getSFVec3f()
valuesR = rotation_field.getSFRotation()
print ("valuesR="+str(valuesR))
print("M1 is at position: %g %g %g" % (values[0], values[1], values[2]))

INITIAL = [0.5, 1.5, 0.0]
trans_field.setSFVec3f(INITIAL)
rotation_field.setSFRotation()

TIME_STEP = 32

INITIAL = [0.5, 1.5, 0.0]
while supervisor.step(TIME_STEP) != -1:
    INITIAL[0] +=0.2
    #print (INITIAL)
    trans_field.setSFVec3f(INITIAL)
    if( INITIAL[0] > 5.0):
        INITIAL[0]=0.5

"""


import numpy as np
import math
def to_axis_angle(quaternion, identity_thresh=None):
    """
    #http://docs.ros.org/en/kinetic/api/baldor/html/_modules/baldor/quaternion.html
    Return axis-angle rotation from a quaternion

    Parameters
    ----------
    quaternion: array_like
      Input quaternion (4 element sequence)
    identity_thresh : None or scalar, optional
      Threshold below which the norm of the vector part of the quaternion (x,
         y, z) is deemed to be 0, leading to the identity rotation.  None (the
         default) leads to a threshold estimated based on the precision of the
         input.

    Returns
    ----------
    axis: array_like
      axis around which rotation occurs
    angle: float
      angle of rotation

    Notes
    -----
    Quaternions :math:`w + ix + jy + kz` are represented as :math:`[w, x, y, z]`.
    A quaternion for which x, y, z are all equal to 0, is an identity rotation.
    In this case we return a `angle=0` and `axis=[1, 0, 0]``. This is an arbitrary
    vector.

    Examples
    --------
    >>> import numpy as np
    >>> import baldor as br
    >>> axis, angle = br.euler.to_axis_angle(0, 1.5, 0, 'szyx')
    >>> np.allclose(axis, [0, 1, 0])
    True
    >>> angle
    1.5
    """
    w, x, y, z = quaternion
    Nq = np.linalg.norm(quaternion)
    if not np.isfinite(Nq):
        return np.array([1.0, 0, 0]), float('nan')
    if identity_thresh is None:
        try:
            identity_thresh = np.finfo(Nq.type).eps * 3
        except (AttributeError, ValueError):  # Not a numpy type or not float
            #_FLOAT_EPS = np.finfo(np.float).eps
            #identity_thresh = br._FLOAT_EPS * 3
            identity_thresh = np.finfo(np.float64).eps*3
    if Nq < np.finfo(np.float64).eps ** 2:  # Results unreliable after normalization
        return np.array([1.0, 0, 0]), 0.0
    if not np.isclose(Nq, 1):  # Normalize if not normalized
        s = math.sqrt(Nq)
        w, x, y, z = w / s, x / s, y / s, z / s
    len2 = x*x + y*y + z*z
    if len2 < identity_thresh**2:
        # if vec is nearly 0,0,0, this is an identity rotation
        return np.array([1.0, 0, 0]), 0.0
    # Make sure w is not slightly above 1 or below -1
    theta = 2 * math.acos(max(min(w, 1), -1))
    return np.array([x, y, z]) / math.sqrt(len2), theta


class MyObjectDriver:
    def init(self, webots_node, properties):
        self.DEBUG=True
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')


        self.myPrint ("MyObjectDriver::webots_node :"+str(webots_node)+"\n")
        self.myPrint ("MyObjectDriver::dir webots_node :"+str(dir(webots_node))+"\n")


        self.__robot = webots_node.robot
        
        

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__target_twist_pose = Twist()



      
        self.__node.create_subscription(Twist, 'obj_cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Twist, 'obj_cmd_pos', self.__cmd_pos_callback, 1)

        try:
            self.__gps = self.__robot.getDevice('gps')
            self.__gps.enable(True)
            
            #self.myPrint ("self.__gps :"+str(self.__gps)+"\n")
            #self.myPrint ("dir self.__gps :"+str(dir(self.__gps))+"\n")
        except:
            self.myPrint ("!!!no gps\n")
            pass

        self.myPrint ("MyObjectDriver::self.__robot :"+str(self.__robot)+"\n")
        self.myPrint ("MyObjectDriver::dir self.__robot :"+str(dir(self.__robot))+"\n")

        #self.myPrint ("self.__robot.sys :"+str(self.__robot.sys)+"\n")
        #self.myPrint ("dir self.__robot.sys :"+str(dir(self.__robot.sys))+"\n")

        #self.myPrint ("self.__robot._Robot__devices :"+str(self.__robot._Robot__devices)+"\n")
        #self.myPrint ("dir self.__robot._Robot__devices :"+str(dir(self.__robot._Robot__devices))+"\n")

        #ok mais sert a rien
        #self.__robot.setCustomData("tata")
        #translation  = self.__robot.getCustomData()
        #self.myPrint ("translation :"+str(translation)+"\n")

        """
        self.supervisor = self.__robot.getSupervisor()
        self.myPrint ("!!!!!!!!!!!!!!!!!!!! self.supervisor :"+str(self.supervisor)+"\n")
        self.myPrint ("(!!!!!!!!!!!!!!!!!!!! dir self.supervisor :"+str(dir(self.supervisor))+"\n")

        self.supervisorInstance = self.__robot.getSupervisorInstance()
        self.myPrint ("!!!!!!!!!!!!!!!!!!!! self.supervisorInstance :"+str(self.supervisorInstance)+"\n")
        self.myPrint ("(!!!!!!!!!!!!!!!!!!!! dir self.supervisorInstance :"+str(dir(self.supervisorInstance))+"\n")

        self.robotR1=self.__robot.getFromDef("R1")
        self.myPrint ("!!!!!!!!!!!!!!!!!!!! self.robotR1 :"+str(self.robotR1)+"\n")
        self.myPrint ("(!!!!!!!!!!!!!!!!!!!! dir self.robotR1 :"+str(dir(self.robotR1))+"\n")

        self.trans_field = self.robotR1.getField("translation")
        self.myPrint ("!!!!!!!!!!!!!!!!!!!! self.trans_field :"+str(self.trans_field)+"\n")

        values = self.trans_field.getSFVec3f()
        self.myPrint("M1 is at position: %g %g %g" % (values[0], values[1], values[2]))
        self.myPrint("\n")
        """

        
        #self.__robot.translation = "0 0 3"
        #myRoot = self.__robot.getRoot()
        #self.myPrint ("myRoot :"+str(myRoot)+"\n")

        #self.my_robot = self.__robot.getDevice('my_robot')#name "mypose" my_robot
        #self.myPrint ("self.__robot :"+str(self.my_robot)+"\n")
        #self.myPrint ("dir self.__robot :"+str(dir(self.my_robot))+"\n")

        #self.mymode =self.__robot.getMode()#name "mypose" my_robot
        #self.myPrint ("self.mymode :"+str(self.mymode)+"\n")
        #self.myPrint ("dir self.mymode :"+str(dir(self.mymode))+"\n")

        self.newSpeed = False
        self.newPose  = False

    def myPrint(self,msg):
        if self.DEBUG:
            self.__node.get_logger().info(msg)

    def __cmd_vel_callback(self, twist):
        self.newSpeed=True
        #self.myPrint ("MyObjectDriver::__cmd_vel_callback call\n")
        self.__target_twist = twist

    def __cmd_pos_callback(self, twist):
        self.newPose=True
        self.__target_twist_pose=twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        #self.myPrint ("MyObjectDriver::step call\n")

        """
        try:
            gpsPose=self.__gps.getValues()
            self.myPrint ("gpsPose :"+str(gpsPose)+"\n")
        except:
            pass
        """

        if self.newSpeed :
            self.newSpeed= False
            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z

            command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
            command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

            self.__left_motor.setVelocity(command_motor_left)
            self.__right_motor.setVelocity(command_motor_right)

        elif self.newPose : 
            self.newPose=False
            self.myPrint ("new pose wanted ="+str( self.__target_twist_pose))
            # self.__target_twist_pose

            robotO1=self.__robot.getFromDef("O1")
            trans_field = robotO1.getField("translation")
            rotation_field = robotO1.getField("rotation")
            valuesXYZ = trans_field.getSFVec3f()
            valuesAxisAngles = rotation_field.getSFRotation()
            self.myPrint ("valuesXYZ="+str(valuesXYZ))
            self.myPrint ("valuesAxisAngles="+str(valuesAxisAngles))

            INITIAL_TX = [self.__target_twist_pose.linear.x, self.__target_twist_pose.linear.y, self.__target_twist_pose.linear.z]
            INITIAL_QUAT=R.from_euler("ZYX",[self.__target_twist_pose.angular.z, self.__target_twist_pose.angular.y, self.__target_twist_pose.angular.x]).as_quat()
            self.myPrint ("INITIAL_QUAT="+str(INITIAL_QUAT))
            axisAnglesVal=to_axis_angle(INITIAL_QUAT)
            trans_field.setSFVec3f(INITIAL_TX)
            valuesAxisAngles[0]=axisAnglesVal[0][0]
            valuesAxisAngles[1]=axisAnglesVal[0][1]
            valuesAxisAngles[2]=axisAnglesVal[0][2]
            valuesAxisAngles[3]=axisAnglesVal[1]
            rotation_field.setSFRotation(valuesAxisAngles)

class MyRobotDriver:


    def init(self, webots_node, properties):
        self.DEBUG=True
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')


        self.myPrint ("webots_node :"+str(webots_node)+"\n")
        self.myPrint ("dir webots_node :"+str(dir(webots_node))+"\n")


        self.__robot = webots_node.robot
        
        

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__target_twist_pose = Twist()



      
        self.__node.create_subscription(Twist, 'robot_cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Twist, 'robot_cmd_pos', self.__cmd_pos_callback, 1)

        try:
            self.__gps = self.__robot.getDevice('gps')
            self.__gps.enable(True)
            
            #self.myPrint ("self.__gps :"+str(self.__gps)+"\n")
            #self.myPrint ("dir self.__gps :"+str(dir(self.__gps))+"\n")
        except:
            self.myPrint ("!!!no gps\n")
            pass

        self.myPrint ("self.__robot :"+str(self.__robot)+"\n")
        self.myPrint ("dir self.__robot :"+str(dir(self.__robot))+"\n")

        #self.myPrint ("self.__robot.sys :"+str(self.__robot.sys)+"\n")
        #self.myPrint ("dir self.__robot.sys :"+str(dir(self.__robot.sys))+"\n")

        #self.myPrint ("self.__robot._Robot__devices :"+str(self.__robot._Robot__devices)+"\n")
        #self.myPrint ("dir self.__robot._Robot__devices :"+str(dir(self.__robot._Robot__devices))+"\n")

        #ok mais sert a rien
        #self.__robot.setCustomData("tata")
        #translation  = self.__robot.getCustomData()
        #self.myPrint ("translation :"+str(translation)+"\n")


        #self.supervisor = self.__robot.getSupervisor()
        #self.myPrint ("self.supervisor :"+str(self.supervisor)+"\n")
        #self.myPrint ("dir self.supervisor :"+str(dir(self.supervisor))+"\n")

        #self.__robot.translation = "0 0 3"
        #myRoot = self.__robot.getRoot()
        #self.myPrint ("myRoot :"+str(myRoot)+"\n")

        #self.my_robot = self.__robot.getDevice('my_robot')#name "mypose" my_robot
        #self.myPrint ("self.__robot :"+str(self.my_robot)+"\n")
        #self.myPrint ("dir self.__robot :"+str(dir(self.my_robot))+"\n")

        #self.mymode =self.__robot.getMode()#name "mypose" my_robot
        #self.myPrint ("self.mymode :"+str(self.mymode)+"\n")
        #self.myPrint ("dir self.mymode :"+str(dir(self.mymode))+"\n")

        self.newSpeed = False
        self.newPose  = False

    def myPrint(self,msg):
        if self.DEBUG:
            self.__node.get_logger().info(msg)

    def __cmd_vel_callback(self, twist):
        self.newSpeed=True
        self.__target_twist = twist

    def __cmd_pos_callback(self, twist):
        self.newPose=True
        self.__target_twist_pose=twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        """
        try:
            gpsPose=self.__gps.getValues()
            self.myPrint ("gpsPose :"+str(gpsPose)+"\n")
        except:
            pass
        """

        if self.newSpeed :
            self.newSpeed= False
            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z

            command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
            command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

            self.__left_motor.setVelocity(command_motor_left)
            self.__right_motor.setVelocity(command_motor_right)

        elif self.newPose : 
            self.newPose=False
            print ("new pose="+str( self.__target_twist_pose))
            # self.__target_twist_pose

"""
def main():
    print('Hi from my_package.')


if __name__ == '__main__':
    main()
"""