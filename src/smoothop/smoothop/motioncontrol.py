import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smoothop_interfaces.msg import MotorMsg, WheelSpeed
import numpy as np
import time

# IN MM
RADIUS = (79.0/2.0)/1000.0
LX = (84.1)/1000.0
LY = (92.5)/1000.0

class MotionController(Node):
    def __init__(self):
        super().__init__("Motion_controller_node")
        
        # Publishers and Subscribers
        self.motorPublisher = self.create_publisher(MotorMsg, '/controlspeed', 10)
        self.cmdSubscriber = self.create_subscription(Twist, '/cmd_vel', self.cmdcallback, 10)
        self.feedbackSub = self.create_subscription(WheelSpeed, '/fb_rot', self.fbCallback, 10)
        self.feedbackPub = self.create_publisher(Twist, '/fb_speed', 10)
        
        self.get_logger().info("Motion controller node has started!")
        
        self.motorMsg = MotorMsg()
        self.feedbackMsg = Twist()

        # Inverse Kinematics Matrix
        self.invKinMatrix = np.array([
            [1, -1, -(LX+LY)],
            [1,  1,  (LX+LY)],
            [1,  1, -(LX+LY)],
            [1, -1,  (LX+LY)]
        ])

        # --- SMOOTHING VARIABLES ---
        # Target velocity (what the controller wants)
        self.target_vel = np.array([[0.0], [0.0], [0.0]])
        # Current velocity (what we are actually sending to the motors)
        self.current_vel = np.array([[0.0], [0.0], [0.0]])
        
        # Acceleration limits (Tune these!)
        # Lower = smoother/slower acceleration
        # Higher = snappier response
        self.MAX_LIN_ACCEL = 1.0  # m/s^2
        self.MAX_ANG_ACCEL = 3.0  # rad/s^2

        self.last_time = time.time()
        
        # Run the control loop at 20Hz (0.05s)
        self.create_timer(0.05, self.control_loop)

    def cmdcallback(self, msg: Twist):
        # We ONLY update the target here. We do not calculate motor speeds yet.
        self.target_vel[0, 0] = msg.linear.x
        self.target_vel[1, 0] = msg.linear.y
        self.target_vel[2, 0] = msg.angular.z

    def control_loop(self):
        # 1. Calculate time step
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # 2. Ramp Current Velocity towards Target Velocity
        diff = self.target_vel - self.current_vel

        # Linear X Ramp
        step_x = self.MAX_LIN_ACCEL * dt
        self.current_vel[0, 0] += np.clip(diff[0, 0], -step_x, step_x)

        # Linear Y Ramp
        step_y = self.MAX_LIN_ACCEL * dt
        self.current_vel[1, 0] += np.clip(diff[1, 0], -step_y, step_y)

        # Angular Z Ramp
        step_z = self.MAX_ANG_ACCEL * dt
        self.current_vel[2, 0] += np.clip(diff[2, 0], -step_z, step_z)

        # 3. Calculate Inverse Kinematics based on RAMPED velocity
        w_speeds = self.inv_kin(self.current_vel)
        
        self.motorMsg.motor1 = w_speeds[0, 0]
        self.motorMsg.motor2 = w_speeds[1, 0]
        self.motorMsg.motor3 = w_speeds[2, 0]
        self.motorMsg.motor4 = w_speeds[3, 0]
        
        self.motorPublisher.publish(self.motorMsg)

    def fbCallback(self, msg: WheelSpeed):
        self.feedbackMsg.linear.x = \
            (msg.fl + msg.fr + msg.rl + msg.rr) * (RADIUS / 4)
        self.feedbackMsg.linear.y = \
            (-msg.fl + msg.fr + msg.rl - msg.rr) * (RADIUS / 4)
        self.feedbackMsg.angular.z = \
            (-msg.fl + msg.fr - msg.rl + msg.rr) * (RADIUS / (4 * (LX + LY)))
        self.feedbackPub.publish(self.feedbackMsg)
        
    def inv_kin(self, smoothOpvel): 
        # Calculate angular velocities of wheels from linear velocity
        w_speeds = np.dot(((self.invKinMatrix) / RADIUS), smoothOpvel)
        maxOmega = np.max(np.abs(w_speeds))

        # Velocity limiting safety
        if maxOmega > 30:
            speedFactor = 30 / maxOmega
            w_speeds = w_speeds * speedFactor
        return w_speeds

def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()