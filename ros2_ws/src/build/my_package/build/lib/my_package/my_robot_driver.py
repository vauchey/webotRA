
import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

"""
/clock
/cmd_vel
/my_robot/Velodyne_VLP_16/point_cloud
/my_robot/camera
/my_robot/camera/camera_info
/my_robot/ds0
/my_robot/ds1
/my_robot/lidar/point_cloud
/parameter_events
/remove_urdf_robot
/rosout
/tf_static

ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
ros2 topic pub /cmd_vel geometry_msgs/Twist  "angular: { z: 0.1 }"
ros2 topic echo /my_robot/Velodyne_VLP_16/point_cloud
ros2 topic echo /my_robot/lidar/point_cloud
 self.__target_twist_pose


ros2 topic pub /cmd_pos geometry_msgs/Twist  "linear: { x: 0.0 ,y: 0.0, z: 2.0 }"
ros2 topic pub /cmd_pos geometry_msgs/Twist  "linear: { x: 0.0 ,y: 0.0, z: 2.0 }, angular: { x: 0.0 ,y: 0.0, z: 0.0  }"
ros2 pub -1 /cmd_po geometry_msgs/Twist -- '[0.0, 0.0, 1.0]' '[0.0, 0.0, 1.8]'
"""

class MyRobotDriver:


    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__target_twist_pose = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Twist, 'cmd_pos', self.__cmd_pos_callback, 1)

        self.__node.get_logger().info("!!!!!!!!!!!!!Vincent")

        self.myPrint ("self.__robot :"+str(self.__robot)+"\n")
        self.myPrint ("dir self.__robot :"+str(dir(self.__robot))+"\n")


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