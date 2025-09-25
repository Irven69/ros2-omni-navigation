import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist
from math import sin, cos, pi
import numpy as np

# This is the node that calculate the position of the robot (odometry) by integrating the linear velocities.
# The TF2 is broadcasted using TransformBroadcaster from tf2_ros

# DO WE NEED JOINTSTATE PUBLISHER ????????????????????????????????????????



class OdomNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Pub, sub, bc
        self.speedSub = self.create_subscription(Twist, '/fb_speed', self.fb_speed_callback, 10)
        self.odomPub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcast = TransformBroadcaster(self)
        self.timer1 = self.create_timer(0.1, self.odom_update)

        # vars
        self.x = 0.0 #dx
        self.y = 0.0 # dy
        self.theta = 0.0 
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.omega_z = 0.0
        self.get_logger().info("Odometry node running (and broadcast TF)")

    def fb_speed_callback(self, msg: Twist):
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.omega_z = msg.angular.z


    def odom_update(self):
        dt = 0.1 #Time dt

        delta_x = (self.vel_x * cos(self.theta) - self.vel_y * sin(self.theta)) * dt
        delta_y = (self.vel_y * cos(self.theta) + self.vel_x * sin(self.theta)) * dt
        delta_theta = self.omega_z * dt

        self.x += delta_x
        self.y += delta_y
        # wrap angle (?)
        self.theta = np.mod((self.theta+delta_theta), 2*pi)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.z = sin(self.theta/2.0)
        odom_msg.pose.pose.orientation.w = cos(self.theta/2.0)

        odom_msg.twist.twist.linear.x = self.vel_x
        odom_msg.twist.twist.linear.y = self.vel_y


        self.odomPub.publish(odom_msg)

        # from code 

        # Broadcast TF2:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        # Check this !!!
        transform.transform.rotation.z = sin(self.theta / 2.0)
        transform.transform.rotation.w = cos(self.theta / 2.0)

        self.tf_broadcast.sendTransform(transform)

""" 
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)
"""



def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdomNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()