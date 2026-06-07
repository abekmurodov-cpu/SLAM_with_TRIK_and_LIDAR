import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import socket
import math

def get_quaternion_from_euler(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

class TrikBridge(Node):
    def __init__(self):
        super().__init__('trik_bridge')
        
        # ROBOT PHYSICAL DIMENSIONS (METERS)
        self.WHEEL_RADIUS = 0.042  # 8.4cm diameter
        self.WHEEL_BASE = 0.300    # 30cm width
        self.GYRO_SCALE = (math.pi / 180.0) / 1000.0 # Convert IMU to Rad/s
        
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Connect to TRIK
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        TRIK_IP = '192.168.50.238' # <--- CHANGE THIS TO YOUR TRIK IP
        self.sock.connect((TRIK_IP, 9090))
        self.get_logger().info("Connected! Calibrating IMU... DO NOT TOUCH ROBOT FOR 1 SECOND!")
        
        # State Variables
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_left_enc = 0; self.last_right_enc = 0
        self.first_read = True
        
        # IMU Calibration
        self.calibrating = True
        self.calib_samples = 0
        self.gyro_bias = 0.0
        
        self.buffer = ""
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.read_sensors)

    def cmd_callback(self, msg):
        speed_scale = 100.0
        turn_scale = 50.0
        left_power = int((msg.linear.x * speed_scale) - (msg.angular.z * turn_scale))
        right_power = int((msg.linear.x * speed_scale) + (msg.angular.z * turn_scale))
        left_power = max(min(left_power, 100), -100)
        right_power = max(min(right_power, 100), -100)
        self.sock.sendall(f"{left_power},{right_power}\n".encode('utf-8'))

    def read_sensors(self):
        try:
            data = self.sock.recv(4096).decode('utf-8')
            if not data: return
            self.buffer += data
            
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                if ':' in line:
                    encoders, scan_data = line.split(':', 1)
                    parts = encoders.split(',')
                    if len(parts) == 3:
                        left_enc, right_enc, raw_gz = map(int, parts)
                        right_enc = -right_enc # Fix for mirrored motors!
                        
                        # CALIBRATION PHASE
                        if self.calibrating:
                            self.gyro_bias += raw_gz
                            self.calib_samples += 1
                            if self.calib_samples >= 20:
                                self.gyro_bias /= 20.0
                                self.calibrating = False
                                self.get_logger().info(f"IMU Calibrated! Bias: {self.gyro_bias}")
                            return 
                        
                        now = self.get_clock().now()
                        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
                        self.last_time = now
                        
                        if self.first_read:
                            self.last_left_enc = left_enc
                            self.last_right_enc = right_enc
                            self.first_read = False
                            
                        # Distance traveled (Forward/Backward from wheels)
                        d_left_deg = left_enc - self.last_left_enc
                        d_right_deg = right_enc - self.last_right_enc
                        self.last_left_enc = left_enc
                        self.last_right_enc = right_enc
                        
                        dist_per_deg = (2.0 * math.pi * self.WHEEL_RADIUS) / 360.0
                        d_center = ((d_left_deg * dist_per_deg) + (d_right_deg * dist_per_deg)) / 2.0
                        
                        # Rotation (Exclusively from the IMU for accuracy)
                        gz_corrected = raw_gz - self.gyro_bias
                        d_theta = (gz_corrected * self.GYRO_SCALE) * dt
                        
                        self.th += d_theta
                        self.x += d_center * math.cos(self.th)
                        self.y += d_center * math.sin(self.th)
                        
                        # Publish TF
                        t = TransformStamped()
                        t.header.stamp = now.to_msg()
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_footprint'
                        t.transform.translation.x = self.x
                        t.transform.translation.y = self.y
                        qx, qy, qz, qw = get_quaternion_from_euler(self.th)
                        t.transform.rotation.x = qx
                        t.transform.rotation.y = qy
                        t.transform.rotation.z = qz
                        t.transform.rotation.w = qw
                        self.tf_broadcaster.sendTransform(t)

                    # Publish LaserScan
                    distances = scan_data.split(',')
                    msg = LaserScan()
                    msg.header.frame_id = 'laser'
                    msg.header.stamp = now.to_msg()
                    msg.angle_min = 0.0
                    msg.angle_max = 2.0 * math.pi
                    if len(distances) > 1:
                        msg.angle_increment = (2.0 * math.pi) / len(distances)
                    msg.range_min = 0.15
                    msg.range_max = 12.0
                    msg.ranges = []
                    for d in distances:
                        try:
                            msg.ranges.append(float(d) / 1000.0) # Convert mm to meters
                        except ValueError:
                            msg.ranges.append(0.0)
                    self.scan_pub.publish(msg)
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TrikBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
