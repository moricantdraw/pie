import serial

# serial constants
PORT = "/dev/ttyACM0"
BAUDRATE = 9600
serialPort = serial.Serial(PORT, BAUDRATE, timeout=1)

# empty arrays for serial data and subsequent plot
ir_vals = []
pt_coords = []
xy_coords = [[], []]

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time


class PIDController:
    def __init__(self, kp, ki, kd):
        # PID coefficients
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def compute(self, setpoint, measured_value):
        # Calculate error
        error = setpoint - measured_value
        
        # Calculate delta time
        current_time = time.time()
        delta_time = current_time - self.prev_time
        
        # Integral and derivative
        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time if delta_time > 0 else 0.0
        
        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Update for next cycle
        self.prev_error = error
        self.prev_time = current_time
        
        return output


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Desired setpoint (could be distance or sensor reading threshold)
        self.setpoint = 1.0
        
        # PID controller (tune these values)
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        
        # Subscriber for the IR sensor data
        self.ir_sensor_subscriber = self.create_subscription(
            Float32,
            'ir_sensor',
            self.ir_sensor_callback,
            10
        )
        
        # Publisher for motor commands (assumes motors take a Twist message)
        self.motor_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize motor control variable
        self.motor_speed = 0.0

    def ir_sensor_callback(self, msg):
        # Get the measured value from the IR sensor
        ir_value = msg.data

        # Compute PID output based on setpoint and sensor value
        control_signal = self.pid.compute(self.setpoint, ir_value)

        # Control the motor speed based on the PID output
        self.motor_speed = max(min(control_signal, 1.0), -1.0)  # constrain the speed between -1.0 and 1.0
        
        # Create Twist message to control motor
        motor_cmd = Twist()
        motor_cmd.linear.x = self.motor_speed
        
        # Publish the motor command
        self.motor_publisher.publish(motor_cmd)
        self.get_logger().info(f'IR Value: {ir_value}, Motor Speed: {self.motor_speed}')


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

