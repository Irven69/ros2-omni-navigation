import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smoothop_interfaces.msg import MotorMsg, WheelSpeed
from sensor_msgs.msg import Joy

# IN MM
RADIUS = (79/2)/1000
LX = (84.1)/1000
LY = (92.5)/1000

class MotionController(Node):
    def __init__(self):
        super().__init__("Motion_controller_node")
        self.motorPublisher = self.create_publisher(MotorMsg, '/controlspeed', 10)
        self.joySubscriber = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        #self.cmdSubscriber = self.create_subscription(Twist, '/cmd_vel' self.cmdcallback, 10)
        self.feedbackSub = self.create_subscription(WheelSpeed, '/fb_rot', self.fbCallback, 10)
        self.feedbackPub = self.create_publisher(Twist, '/fb_speed', 10)
        self.get_logger().info("Motion controller node has started!")
        self.motorMsg = MotorMsg()
        self.feedbackMsg = Twist()

    def joycallback(self, msg: Joy):
        x_vel = 0.5* msg.axes[1] # m/s
        y_vel = 0.5* msg.axes[0]
        w_z = 3* msg.axes[3] #rad/s

        speeds = self.inv_kin(x_vel, y_vel, w_z)
        self.motorMsg.motor1 = speeds[0]
        self.motorMsg.motor2 = speeds[1]
        self.motorMsg.motor3 = speeds[2]
        self.motorMsg.motor4 = speeds[3]
        self.motorPublisher.publish(self.motorMsg)

    def cmdcallback(self, msg: Twist):
        x_vel = msg.linear.x # m/s
        y_vel = msg.linear.y
        w_z = msg.angular.z #rad/s
        speeds = self.inv_kin(x_vel, y_vel, w_z)
        self.motorMsg.motor1 = speeds[0]
        self.motorMsg.motor2 = speeds[1]
        self.motorMsg.motor3 = speeds[2]
        self.motorMsg.motor4 = speeds[3]
        self.motorPublisher.publish(self.motorMsg)



    def fbCallback(self, msg: WheelSpeed):
        self.feedbackMsg.linear.x = \
            (msg.fl+msg.fr+msg.rl+msg.rr)*(RADIUS/4)
        self.feedbackMsg.linear.y = \
            (-msg.fl+msg.fr+msg.rl-msg.rr)*(RADIUS/4)
        self.feedbackMsg.angular.z = \
            (-msg.fl+msg.fr-msg.rl+msg.rr)*(RADIUS/(4*(LX+LY)))
        self.feedbackPub.publish(self.feedbackMsg)
        
        
    def inv_kin(self, x_vel, y_vel, w_z): #Calculate angular velocities of wheels from linear velocity by joy
        w_fl = (1/RADIUS)*(x_vel-y_vel-(LX+LY)*w_z) # motor 1
        w_fr = (1/RADIUS)*(x_vel+y_vel+(LX+LY)*w_z) # motor 2
        w_rl = (1/RADIUS)*(x_vel+y_vel-(LX+LY)*w_z) # motor 3
        w_rr = (1/RADIUS)*(x_vel-y_vel+(LX+LY)*w_z) # motor 4
        speeds = [w_fl, w_fr, w_rl, w_rr]
        return speeds
        




def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
