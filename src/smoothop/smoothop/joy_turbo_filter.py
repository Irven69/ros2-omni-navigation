import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyTurboFilter(Node):
    def __init__(self):
        super().__init__('joy_turbo_filter')
        
        # Declare all parameters with defaults
        self.declare_parameter('enable_button', 10)
        self.declare_parameter('turbo_button', 7)
        self.declare_parameter('dpad_speed', 0.5)
        self.declare_parameter('dpad_speed_turbo', 0.8)
        self.declare_parameter('dpad_up_button', 11)
        self.declare_parameter('dpad_down_button', 12)
        self.declare_parameter('dpad_left_button', 13)
        self.declare_parameter('dpad_right_button', 14)
        
        # Get parameter values
        self.enable_idx = self.get_parameter('enable_button').value
        self.turbo_idx = self.get_parameter('turbo_button').value
        self.dpad_speed = self.get_parameter('dpad_speed').value
        self.dpad_speed_turbo = self.get_parameter('dpad_speed_turbo').value
        self.dpad_up_idx = self.get_parameter('dpad_up_button').value
        self.dpad_down_idx = self.get_parameter('dpad_down_button').value
        self.dpad_left_idx = self.get_parameter('dpad_left_button').value
        self.dpad_right_idx = self.get_parameter('dpad_right_button').value
        
        # Create publishers and subscribers
        self.sub = self.create_subscription(Joy, 'joy', self.cb, 10)
        self.pub = self.create_publisher(Joy, 'joy_filtered', 10)
        
        self.get_logger().info(
            f'Joy filter: enable={self.enable_idx}, turbo={self.turbo_idx}, '
            f'dpad=[{self.dpad_up_idx},{self.dpad_down_idx},{self.dpad_left_idx},{self.dpad_right_idx}]'
        )

    def cb(self, msg: Joy):
        # Create output message
        out_msg = Joy()
        out_msg.header = msg.header
        out_msg.axes = list(msg.axes)
        out_msg.buttons = list(msg.buttons)
        
        # Safety check
        if len(out_msg.buttons) < max(self.enable_idx, self.turbo_idx, 
                                       self.dpad_up_idx, self.dpad_down_idx,
                                       self.dpad_left_idx, self.dpad_right_idx) + 1:
            self.pub.publish(out_msg)
            return
        
        if len(out_msg.axes) < 4:
            self.pub.publish(out_msg)
            return
        
        # Check button states
        enable_pressed = bool(out_msg.buttons[self.enable_idx])
        turbo_pressed = bool(out_msg.buttons[self.turbo_idx])
        
        # Clear turbo if enable not pressed
        if not enable_pressed:
            out_msg.buttons[self.turbo_idx] = 0
        
        # D-pad button processing (only if enable pressed)
        if enable_pressed:
            dpad_up = bool(out_msg.buttons[self.dpad_up_idx])
            dpad_down = bool(out_msg.buttons[self.dpad_down_idx])
            dpad_left = bool(out_msg.buttons[self.dpad_left_idx])
            dpad_right = bool(out_msg.buttons[self.dpad_right_idx])
            
            # Select speed based on turbo
            speed = self.dpad_speed_turbo if turbo_pressed else self.dpad_speed
            
            # Override axis 1 (forward/backward) with D-pad up/down
            if dpad_up:
                out_msg.axes[1] = speed  # up = positive (forward)
            elif dpad_down:
                out_msg.axes[1] = -speed  # down = negative (backward)
            
            # Override axis 0 (strafe left/right) with D-pad left/right
            if dpad_left:
                out_msg.axes[0] = speed  # left = positive (strafe left)
            elif dpad_right:
                out_msg.axes[0] = -speed  # right = negative (strafe right)
        
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTurboFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()