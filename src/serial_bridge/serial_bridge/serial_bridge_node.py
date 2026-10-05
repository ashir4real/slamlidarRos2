#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import sys

def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', False)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value

        self.ser = None
        if not self.simulate:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'Serial connected to {port}')
                time.sleep(2)
                self.ser.reset_input_buffer()
            except Exception as e:
                self.get_logger().error(f'Failed to open serial: {e}')
                sys.exit(1)
        else:
            self.get_logger().info('Running in simulation mode')

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Read serial at 50 Hz
        self.create_timer(0.02, self.read_serial)
        # Publish odom + TF at 20 Hz
        self.create_timer(0.05, self.publish_odometry)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.sim_linear = 0.0
        self.sim_angular = 0.0

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        self.last_linear = linear
        self.last_angular = angular
        cmd_str = f"v:{linear:.2f} r:{angular:.2f}\n"
        if self.ser:
            self.ser.write(cmd_str.encode())
        else:
            self.sim_linear = linear
            self.sim_angular = angular

    def read_serial(self):
        if self.ser and self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if not line:
                return
            if 'x:' in line and 'y:' in line and 'theta:' in line:
                try:
                    parts = line.split()
                    data = {}
                    for part in parts:
                        if ':' not in part:
                            continue
                        key, val = part.split(':')
                        data[key] = float(val)
                    if 'x' in data and 'y' in data and 'theta' in data:
                        self.x = data['x']
                        self.y = data['y']
                        self.theta = data['theta']
                except Exception as e:
                    self.get_logger().warn(f'Parse error: {line} - {e}')
        elif self.simulate:
            dt = 0.02
            linear = self.sim_linear
            angular = self.sim_angular
            if abs(angular) < 1e-6:
                self.x += linear * dt * math.cos(self.theta)
                self.y += linear * dt * math.sin(self.theta)
            else:
                radius = linear / angular
                self.theta += angular * dt
                self.x += radius * (math.sin(self.theta) - math.sin(self.theta - angular * dt))
                self.y -= radius * (math.cos(self.theta) - math.cos(self.theta - angular * dt))
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(self.theta)
        odom.pose.covariance = [0.01] * 36

        odom.twist.twist.linear.x = self.last_linear
        odom.twist.twist.angular.z = self.last_angular
        odom.twist.covariance = [0.01] * 36

        self.odom_pub.publish(odom)

        # Broadcast TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self.theta)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
