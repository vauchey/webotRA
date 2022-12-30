
import rclpy
from geometry_msgs.msg import Twist

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

#https://cyberbotics.com/doc/guide/supervisor-programming?tab-language=python
ros2 topic pub /robot_cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
ros2 topic pub /robot_cmd_vel geometry_msgs/Twist  "angular: { z: 0.1 }"
ros2 topic echo /my_robot/Velodyne_VLP_16/point_cloud
ros2 topic echo /my_robot/lidar/point_cloud
 self.__target_twist_pose


ros2 topic pub /robot_cmd_pos geometry_msgs/Twist  "linear: { x: 0.0 ,y: 0.0, z: 2.0 }"
ros2 topic pub /robot_cmd_pos geometry_msgs/Twist  "linear: { x: 0.0 ,y: 0.0, z: 2.0 }, angular: { x: 0.0 ,y: 0.0, z: 0.0  }"
ros2 pub -1 /cmd_po geometry_msgs/Twist -- '[0.0, 0.0, 1.0]' '[0.0, 0.0, 1.8]'
"""


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
            print ("new pose="+str( self.__target_twist_pose))
            # self.__target_twist_pose

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