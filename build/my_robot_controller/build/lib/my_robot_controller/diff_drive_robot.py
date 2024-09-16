#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Int16, Float32, Int32
import serial
import threading

from my_robot_controller.submodules.pid import PID
from my_robot_controller.submodules.encoder_to_wheel_jointstate import WheelState
from my_robot_controller.submodules.encoder_wrap import EncoderWrap

class DiffDriveRobot(Node):
    def __init__(self):
        super().__init__('differential_drive_robot')
        
        # Robot parameters
        self.ticks_meter = 74800
        self.wheel_one_rev_ticks = 47000
        self.base_width = 0.28
        self.wheel_radius = 0.10  # Wheel radius in meters
        self.base_frame_id = "base_footprint"
        self.odom_frame_id = "odom"
        
        # Serial communication
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.lock = threading.Lock()
        
        # Encoder objects
        self.left_enc = EncoderWrap(encoder_min=-32768, encoder_max=32767)
        self.right_enc = EncoderWrap(encoder_min=-32768, encoder_max=32767)
        
        # PID controllers
        self.left_pid = PID(Kp=200, Ki=90, Kd=1.0, highest_pwm=150, lowest_pwm=60)
        self.right_pid = PID(Kp=200, Ki=90, Kd=1.0, highest_pwm=150, lowest_pwm=60)
        
        # Wheel state objects
        self.left_wheel = WheelState(radian_per_rotate=2*pi, enc_tick_rotate=self.wheel_one_rev_ticks, float_round=2)
        self.right_wheel = WheelState(radian_per_rotate=2*pi, enc_tick_rotate=self.wheel_one_rev_ticks, float_round=2)
        
        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0
        self.dr = 0.0
        self.left_wheel_pwm = 0
        self.right_wheel_pwm = 0
        self.target_left_velocity = 0.0
        self.target_right_velocity = 0.0
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0
        self.commanded_linear_velocity = 0.0
        self.commanded_angular_velocity = 0.0
        
        # Initialize last_time and encoder values
        self.last_time = self.get_clock().now()
        self.last_left_encoder = 0
        self.last_right_encoder = 0
                
        # ROS publishers and subscribers
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_pwm_pub = self.create_publisher(Int16, 'motor_pwm', 10)
        self.motor_target_vel_pub = self.create_publisher(Float32, 'target_vel', 10)
        self.motor_current_vel_pub = self.create_publisher(Float32, 'current_vel', 10)
        
        # Publishers for encoder readings
        self.left_encoder_pub = self.create_publisher(Int32, 'left_encoder', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'right_encoder', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.create_timer(0.01, self.update)
        self.create_timer(0.05, self.target_wheel_velocity)
        self.create_timer(0.001, self.send_receive_data)
        self.create_timer(1.0, self.show_data)
        
        self.get_logger().info("Two-wheel Differential Drive Robot initialized")

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return  # Avoid division by zero
        
        # Read encoder data
        left_enc, right_enc = self.read_encoders()
        
        # Publish encoder readings
        self.publish_encoder_readings(left_enc, right_enc)
        
        # Calculate odometry
        delta_left = left_enc - self.last_left_encoder
        delta_right = right_enc - self.last_right_encoder
        
        d_left = (delta_left / self.wheel_one_rev_ticks) * (2 * pi * self.wheel_radius)
        d_right = (delta_right / self.wheel_one_rev_ticks) * (2 * pi * self.wheel_radius)
        
        # Update wheel states
        left_pos = self.left_wheel.get_state(left_enc, self.last_left_encoder)
        right_pos = self.right_wheel.get_state(right_enc, self.last_right_encoder)
        
        self.last_left_encoder = left_enc
        self.last_right_encoder = right_enc
        
        d = (d_left + d_right) / 2
        th = (d_right - d_left) / self.base_width
        
        # Modified odometry calculation
        self.x -= d * cos(self.th)
        self.y -= d * sin(self.th)
        self.th -= th
        
        # Calculate velocities
        self.current_left_velocity = d_left / dt
        self.current_right_velocity = d_right / dt
        self.dx = d / dt
        self.dr = th / dt
        
        # Update PID controllers
        self.left_wheel_pwm = self.left_pid.get_pid_output(dt, self.target_left_velocity, self.current_left_velocity)
        self.right_wheel_pwm = self.right_pid.get_pid_output(dt, self.target_right_velocity, self.current_right_velocity)
        
        # Publish odometry and joint states
        self.publish_odometry(now)
        self.publish_joint_states(now, left_pos, right_pos)
        
        self.last_time = now

    def target_wheel_velocity(self):
        self.target_left_velocity = self.commanded_linear_velocity - (self.commanded_angular_velocity * self.base_width / 2)
        self.target_right_velocity = self.commanded_linear_velocity + (self.commanded_angular_velocity * self.base_width / 2)

    def send_receive_data(self):
        with self.lock:
            # Prepare data to send to Arduino
            left_dir = 'F' if self.target_left_velocity >= 0 else 'R'
            right_dir = 'F' if self.target_right_velocity >= 0 else 'R'
            send_data = f'K{left_dir}{abs(int(self.left_wheel_pwm)):03d}{right_dir}{abs(int(self.right_wheel_pwm)):03d}G'
            
            # Send data to Arduino
            self.ser.write(send_data.encode())
            
            # Receive data from Arduino
            received_data = self.ser.readline().decode().strip()
        return received_data

    def read_encoders(self):
        data = self.send_receive_data()
        try:
            right_enc, left_enc = map(int, data.split(','))
            return self.left_enc.get_enc_tick(left_enc), self.right_enc.get_enc_tick(right_enc)
        except ValueError:
            self.get_logger().warn(f"Failed to parse encoder data: {data}")
            return self.last_left_encoder, self.last_right_encoder

    def publish_encoder_readings(self, left_enc, right_enc):
        left_msg = Int32()
        left_msg.data = left_enc
        self.left_encoder_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = right_enc
        self.right_encoder_pub.publish(right_msg)

    def show_data(self):
        self.get_logger().info(f"Left: PWM={self.left_wheel_pwm:.2f}, Target={self.target_left_velocity:.2f}, Current={self.current_left_velocity:.2f}")
        self.get_logger().info(f"Right: PWM={self.right_wheel_pwm:.2f}, Target={self.target_right_velocity:.2f}, Current={self.current_right_velocity:.2f}")

    def twist_callback(self, msg):
        self.commanded_linear_velocity = msg.linear.x
        self.commanded_angular_velocity = msg.angular.z

    def publish_odometry(self, now):
        quat = self.euler_to_quaternion(0, 0, self.th)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)

    def publish_joint_states(self, now, left_pos, right_pos):
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [-left_pos, -right_pos]
        self.joint_pub.publish(joint_state)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

