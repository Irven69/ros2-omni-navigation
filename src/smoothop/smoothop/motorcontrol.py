import rclpy
from rclpy.node import Node
from smoothop_interfaces.msg import MotorMsg, WheelSpeed
import serial
import time

DEVICE = '/dev/ttyACM0'
BAUDRATE = 115200
SF = 255.0/30.0
SF_REV = 30.0/255.0


class MotorController(Node):
    def __init__(self):
        super().__init__("Motor_controller")
        self.subscriber = self.create_subscription(MotorMsg, '/controlspeed', self.setspeeds, 10)
        self.feedbackPub = self.create_publisher(WheelSpeed, '/fb_rot', 10)
        self.serialcb = self.create_timer(0.01, self.serialcb)
        self.ser = serial.Serial(DEVICE, BAUDRATE, timeout=1)
        time.sleep(2) # Wait for serial connection
        self.get_logger().info("Motor controller node running!")
        self.wheelSpeedMSg = WheelSpeed()
        self.serial_msg = bytearray()
    
    def create_binary_msg(self, direction: list[int] , speed: list[int]):
        if len(direction) != 4 or len(speed) != 4:
            raise ValueError("Speed and direction can only contain 4 values")
        msg = bytearray()
        # Creating a byte that contains the 4 directions (4 binary values) using bitshifting
        direction_byte = (direction[3] << 3) | (direction[2] << 2) | (direction[1] << 1) | (direction[0] << 0)
        speedByte0 = speed[0].to_bytes(1) # Creating bytes that contain 4 motor speeds
        speedByte1 = speed[1].to_bytes(1)
        speedByte2 = speed[2].to_bytes(1)
        speedByte3 = speed[3].to_bytes(1)
        # Creating message (bytearray)
        msg.append(direction_byte)
        msg.extend(speedByte0)
        msg.extend(speedByte1)
        msg.extend(speedByte2)
        msg.extend(speedByte3)
        return msg
    
    def decode_binary_msg(self , msg: bytearray):
        speed = [0]*4
        direction = [0]*4
        wheelspeed = [0]*4
        directionByte = msg[0]
        for i in range(4):
            speed[i] = int(msg[i+1])
            direction[i] = (directionByte >> i) & 1
            if direction[i] == 1: wheelspeed[i] = speed[i]*SF_REV
            else: wheelspeed[i] = -speed[i]*SF_REV
        return wheelspeed

    def setspeeds(self, msg: MotorMsg):
        # Scaling motor speeds from angular velocity (float -20 <> 20) to uint8 and direction
        # 0=0 and 255 = 20
        # SF = scaling factor = 255/20
        speed1 = int(abs(msg.motor1*SF))
        speed2 = int(abs(msg.motor2*SF))
        speed3 = int(abs(msg.motor3*SF))
        speed4 = int(abs(msg.motor4*SF))

        self.ser.reset_input_buffer()

        dir1 = 0 if msg.motor1 < 0 else 1
        dir2 = 0 if msg.motor2 < 0 else 1
        dir3 = 0 if msg.motor3 < 0 else 1
        dir4 = 0 if msg.motor4 < 0 else 1

        speeds = [speed1, speed2, speed3, speed4]
        direction = [dir1, dir2, dir3, dir4]
        try:
            self.serial_msg = self.create_binary_msg(direction, speeds)
            #self.ser.write(self.serial_msg)
        except:
            self.get_logger().warning("Bin except")
        #self.ser.reset_output_buffer()
        #time.sleep(0.001)
        #print("Message sent:", serial_msg.hex())

    def serialcb(self):
        binmsg = None
        try:
            self.ser.write(self.serial_msg)
            time.sleep(0.001)
        except:
            pass
        try:
            binmsg = bytearray(self.ser.read(5))
        except:
            #self.get_logger().warning("Serial except")
            pass
        try:
            speeds = self.decode_binary_msg(binmsg)
            self.wheelSpeedMSg.fl = speeds[0] #motor 1
            self.wheelSpeedMSg.fr = speeds[1] #motor 2
            self.wheelSpeedMSg.rl = speeds[2] #motor 3
            self.wheelSpeedMSg.rr = speeds[3] #motor 4
            # print("Message received:", binmsg.hex())
            self.feedbackPub.publish(self.wheelSpeedMSg)
        except:
            #self.get_logger().warning("decode except")
            pass


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.ser.close()
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()