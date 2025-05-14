#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from transforms3d.euler import euler2quat
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SimulatedRobot(Node):
    def __init__(self):
        super().__init__('simulated_robot')
        
        # 初始化参数
        self.x = 0.0  # x位置 (m)
        self.y = 0.0  # y位置 (m)
        self.yaw = 0.0  # 朝向 (rad)
        self.last_time = self.get_clock().now()
        
        # 噪声参数
        self.linear_noise_stddev = 0.02  # 线速度噪声标准差 (m/s)
        self.angular_noise_stddev = 0.01  # 角速度噪声标准差 (rad/s)
        self.position_noise_stddev = 0.01  # 位置噪声标准差 (m)
        
        # 设置QoS配置以确保可靠传输
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # 订阅cmd_vel话题
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布Odometry话题
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            qos_profile
        )
        
        # 创建10Hz的定时器
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10Hz
        
        self.get_logger().info("Simulated robot node initialized")

    def cmd_vel_callback(self, msg):
        """处理速度指令"""
        # 添加高斯噪声
        noisy_linear_x = msg.linear.x + np.random.normal(0, self.linear_noise_stddev)
        noisy_angular_z = msg.angular.z + np.random.normal(0, self.angular_noise_stddev)
        
        # 更新位置 (简单的欧拉积分)
        current_time = self.get_clock().now()
        dt = 0.01
        self.last_time = current_time
        
        self.x += noisy_linear_x * np.cos(self.yaw) * dt
        self.y += noisy_linear_x * np.sin(self.yaw) * dt
        self.yaw += noisy_angular_z * dt
        
        # 归一化角度到[-pi, pi]
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))

    def update_odometry(self):
        """定时发布Odometry消息"""
        # 添加位置噪声
        noisy_x = self.x + np.random.normal(0, self.position_noise_stddev)
        noisy_y = self.y + np.random.normal(0, self.position_noise_stddev)
        
        # 创建Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # 设置位置
        odom_msg.pose.pose.position.x = noisy_x
        odom_msg.pose.pose.position.y = noisy_y
        odom_msg.pose.pose.position.z = 0.0
        
        # 设置朝向 (转换为四元数)
        q = euler2quat(0, 0, self.yaw)  # 顺序为roll, pitch, yaw
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 设置协方差 (简化模型)
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,   # y
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,   # z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,   # roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,   # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01    # yaw
        ]
        
        # 发布消息
        self.odom_pub.publish(odom_msg)
        
        # 打印调试信息
        self.get_logger().debug(
            f"Publishing odometry: x={noisy_x:.2f}, y={noisy_y:.2f}, yaw={self.yaw:.2f}",
            throttle_duration_sec=1.0  # 限制日志频率
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()