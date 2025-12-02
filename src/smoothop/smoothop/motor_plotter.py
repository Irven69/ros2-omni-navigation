#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smoothop_interfaces.msg import MotorMsg, WheelSpeed
from std_msgs.msg import Float32

class MotorPlotter(Node):
    def __init__(self):
        super().__init__('motor_plotter')
        
        # Subscribers
        self.sub_cmd = self.create_subscription(
            MotorMsg, '/controlspeed', self.cmd_cb, 10)
        self.sub_fb = self.create_subscription(
            WheelSpeed, '/fb_rot', self.fb_cb, 10)
        
        # Publishers - use Float32 with valid topic names (motor names: fl, fr, rl, rr)
        motor_names = ['fl', 'fr', 'rl', 'rr']
        
        self.pub_setpoint = [
            self.create_publisher(Float32, f'/motor/setpoint/{name}', 10) 
            for name in motor_names
        ]
        self.pub_actual = [
            self.create_publisher(Float32, f'/motor/actual/{name}', 10) 
            for name in motor_names
        ]
        self.pub_error = [
            self.create_publisher(Float32, f'/motor/error/{name}', 10) 
            for name in motor_names
        ]
        
        self.setpoints = [0.0, 0.0, 0.0, 0.0]
        self.actuals = [0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('Motor plotter node started')
    
    def cmd_cb(self, msg):
        self.setpoints = [msg.motor1, msg.motor2, msg.motor3, msg.motor4]
        self.publish_data()
    
    def fb_cb(self, msg):
        self.actuals = [msg.fl, msg.fr, msg.rl, msg.rr]
        self.publish_data()
    
    def publish_data(self):
        for i in range(4):
            # Publish setpoint
            sp_msg = Float32()
            sp_msg.data = float(self.setpoints[i])
            self.pub_setpoint[i].publish(sp_msg)
            
            # Publish actual
            act_msg = Float32()
            act_msg.data = float(self.actuals[i])
            self.pub_actual[i].publish(act_msg)
            
            # Publish error
            err_msg = Float32()
            err_msg.data = float(self.setpoints[i] - self.actuals[i])
            self.pub_error[i].publish(err_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()