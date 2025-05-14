#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QWidget, 
                            QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                            QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QDoubleValidator
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class PlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_pos = (0, 0)
        self.target_pos = None
        self.point1 = None
        self.point2 = None
        self.setMinimumSize(500, 500)
        
    def update_positions(self, current, target, point1=None, point2=None):
        self.current_pos = current
        self.target_pos = target
        self.point1 = point1
        self.point2 = point2
        self.update()  # 触发重绘
        
    def paintEvent(self, event):
        """绘制坐标系和位置"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 获取尺寸
        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2
        
        # 绘制背景和网格
        painter.fillRect(event.rect(), QColor(240, 240, 240))
        
        # 绘制网格线
        grid_pen = QPen(QColor(200, 200, 200), 1, Qt.DotLine)
        painter.setPen(grid_pen)
        grid_size = 50
        for x in range(0, width, grid_size):
            painter.drawLine(x, 0, x, height)
        for y in range(0, height, grid_size):
            painter.drawLine(0, y, width, y)
        
        # 绘制坐标轴
        axis_pen = QPen(Qt.black, 2)
        painter.setPen(axis_pen)
        painter.drawLine(0, center_y, width, center_y)  # X轴
        painter.drawLine(center_x, 0, center_x, height)  # Y轴
        
        # 绘制刻度
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        for i in range(-10, 11, 2):
            if i == 0: continue
            # X轴刻度
            x_pos = center_x + int(i * (width / 20))
            painter.drawLine(x_pos, center_y-5, x_pos, center_y+5)
            painter.drawText(x_pos-10, center_y+20, str(i))
            # Y轴刻度
            y_pos = center_y - int(i * (height / 20))
            painter.drawLine(center_x-5, y_pos, center_x+5, y_pos)
            painter.drawText(center_x+10, y_pos+5, str(i))
        
        # 绘制原点标记
        painter.drawText(center_x+5, center_y+20, "0")
        
        # 绘制当前位置（蓝色圆点）
        if hasattr(self, 'current_pos'):
            x, y = self.current_pos
            norm_x = center_x + x * (width / 20)
            norm_y = center_y - y * (height / 20)
            painter.setPen(QPen(Qt.blue, 2))
            painter.setBrush(QBrush(Qt.blue))
            painter.drawEllipse(int(norm_x-5), int(norm_y-5), 10, 10)
        
        # 绘制目标位置（红色圆点）
        if self.target_pos:
            x, y = self.target_pos
            norm_x = center_x + x * (width / 20)
            norm_y = center_y - y * (height / 20)
            painter.setPen(QPen(Qt.red, 2))
            painter.setBrush(QBrush(Qt.red))
            painter.drawEllipse(int(norm_x-5), int(norm_y-5), 10, 10)
            
            # 绘制轨迹线
            if hasattr(self, 'current_pos'):
                curr_x = center_x + self.current_pos[0] * (width / 20)
                curr_y = center_y - self.current_pos[1] * (height / 20)
                painter.setPen(QPen(Qt.green, 2, Qt.DashLine))
                painter.drawLine(int(curr_x), int(curr_y), int(norm_x), int(norm_y))
        
        # 绘制点1（紫色圆点）
        if self.point1:
            x, y = self.point1
            norm_x = center_x + x * (width / 20)
            norm_y = center_y - y * (height / 20)
            painter.setPen(QPen(QColor(128, 0, 128), 2))  # 紫色
            painter.setBrush(QBrush(QColor(128, 0, 128)))
            painter.drawEllipse(int(norm_x-5), int(norm_y-5), 10, 10)
            painter.drawText(int(norm_x+10), int(norm_y), "Point 1")
        
        # 绘制点2（橙色圆点）
        if self.point2:
            x, y = self.point2
            norm_x = center_x + x * (width / 20)
            norm_y = center_y - y * (height / 20)
            painter.setPen(QPen(QColor(255, 165, 0), 2))  # 橙色
            painter.setBrush(QBrush(QColor(255, 165, 0)))
            painter.drawEllipse(int(norm_x-5), int(norm_y-5), 10, 10)
            painter.drawText(int(norm_x+10), int(norm_y), "Point 2")

class NavigationUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ROS2 Navigation UI")
        self.setGeometry(100, 100, 800, 600)
        
        # 初始化变量
        self.current_pos = (0.0, 0.0)
        self.target_pos = None
        self.point1 = None
        self.point2 = None
        self.max_speed = 0.5
        self.coord_range = (-10, 10)  # 坐标系范围
        
        # 创建UI元素
        self.init_ui()
        
        # 订阅Odometry话题
        self.ros_node.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # 订阅cmd_vel话题
        self.cmd_vel_sub = self.ros_node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 订阅两个额外的坐标点话题
        self.point1_sub = self.ros_node.create_subscription(
            Point,
            'point1',
            self.point1_callback,
            10
        )
        
        self.point2_sub = self.ros_node.create_subscription(
            Point,
            'point2',
            self.point2_callback,
            10
        )
        
        # 创建定时器更新UI
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(100)  # 10Hz更新
        
    def init_ui(self):
        # 主布局
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # 左侧绘图区域
        self.plot_widget = PlotWidget()
        self.plot_widget.setMinimumSize(500, 500)
        self.plot_widget.mousePressEvent = self.plot_clicked
        
        # 右侧控制面板
        control_panel = QGroupBox("Control Panel")
        control_layout = QVBoxLayout()
        
        # 当前位置显示
        self.pos_label = QLabel("Current Position: (0.00, 0.00)")
        control_layout.addWidget(self.pos_label)
        
        # 目标位置显示
        self.target_label = QLabel("Target Position: Not set")
        control_layout.addWidget(self.target_label)
        
        # 点1位置显示
        self.point1_label = QLabel("Point 1: Not received")
        control_layout.addWidget(self.point1_label)
        
        # 点2位置显示
        self.point2_label = QLabel("Point 2: Not received")
        control_layout.addWidget(self.point2_label)
        
        # 最大速度设置
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Max Speed (m/s):"))
        self.speed_input = QLineEdit(str(self.max_speed))
        self.speed_input.setValidator(QDoubleValidator(0.1, 10.0, 2))
        speed_layout.addWidget(self.speed_input)
        
        self.set_speed_btn = QPushButton("Set Speed")
        self.set_speed_btn.clicked.connect(self.set_max_speed)
        speed_layout.addWidget(self.set_speed_btn)
        control_layout.addLayout(speed_layout)
        
        # 设置目标按钮
        self.set_target_btn = QPushButton("Set Target")
        self.set_target_btn.clicked.connect(self.send_target)
        self.set_target_btn.setEnabled(False)
        control_layout.addWidget(self.set_target_btn)
        
        self.cmd_vel_group = QGroupBox("Command Velocity Info")
        cmd_vel_layout = QVBoxLayout()

        # 线速度显示
        self.linear_vel_label = QLabel("Linear:  x: 0.00\ny: 0.00")
        # 角速度显示
        self.angular_vel_label = QLabel("Angular: z: 0.00")

        cmd_vel_layout.addWidget(self.linear_vel_label)
        cmd_vel_layout.addWidget(self.angular_vel_label)
        self.cmd_vel_group.setLayout(cmd_vel_layout)
        control_layout.addWidget(self.cmd_vel_group)
        # 添加伸缩空间
        control_layout.addStretch()
        
        control_panel.setLayout(control_layout)
        
        # 组合布局
        main_layout.addWidget(self.plot_widget)
        main_layout.addWidget(control_panel)
        main_widget.setLayout(main_layout)
        
        self.setCentralWidget(main_widget)
    
    def cmd_vel_callback(self, msg):
        """更新cmd_vel显示"""
        self.linear_vel_label.setText(
            f"Linear:  x: {msg.linear.x:.2f}\n"
            f"        y: {msg.linear.y:.2f}"
        )
        self.angular_vel_label.setText(
            f"Angular: z: {msg.angular.z:.2f}"
        )
    
    def odom_callback(self, msg):
        """处理Odometry消息，更新当前位置"""
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.plot_widget.update_positions(self.current_pos, self.target_pos, self.point1, self.point2)
    
    def point1_callback(self, msg):
        """处理点1的消息"""
        self.point1 = (msg.x, msg.y)
        self.point1_label.setText(f"Point 1: ({msg.x:.2f}, {msg.y:.2f})")
        self.plot_widget.update_positions(self.current_pos, self.target_pos, self.point1, self.point2)
    
    def point2_callback(self, msg):
        """处理点2的消息"""
        self.point2 = (msg.x, msg.y)
        self.point2_label.setText(f"Point 2: ({msg.x:.2f}, {msg.y:.2f})")
        self.plot_widget.update_positions(self.current_pos, self.target_pos, self.point1, self.point2)
    
    def plot_clicked(self, event):
        """处理绘图区域的鼠标点击事件"""
        if event.button() == Qt.LeftButton:
            # 将点击位置转换为坐标系中的坐标
            width = self.plot_widget.width()
            height = self.plot_widget.height()
            
            # 归一化坐标 (0-1)
            x_norm = event.x() / width
            y_norm = 1.0 - (event.y() / height)  # 反转Y轴
            
            # 转换为实际坐标
            x = self.coord_range[0] + x_norm * (self.coord_range[1] - self.coord_range[0])
            y = self.coord_range[0] + y_norm * (self.coord_range[1] - self.coord_range[0])
            
            self.target_pos = (x, y)
            self.target_label.setText(f"Target Position: ({x:.2f}, {y:.2f})")
            self.set_target_btn.setEnabled(True)
            self.plot_widget.update_positions(self.current_pos, self.target_pos, self.point1, self.point2)
    
    def send_target(self):
        """发送目标位置到ROS话题"""
        if self.target_pos:
            target_msg = Point()
            target_msg.x = self.target_pos[0]
            target_msg.y = self.target_pos[1]
            
            # 发布目标位置
            pub = self.ros_node.create_publisher(Point, 'ui_target_position', 10)
            pub.publish(target_msg)
            self.ros_node.get_logger().info(f"Published target: {target_msg.x}, {target_msg.y}")
    
    def set_max_speed(self):
        """设置最大速度"""
        try:
            speed = float(self.speed_input.text())
            if 0.1 <= speed <= 10.0:
                self.max_speed = speed
                
                # 发布最大速度
                speed_msg = Float32()
                speed_msg.data = self.max_speed
                pub = self.ros_node.create_publisher(Float32, 'ui_max_speed', 10)
                pub.publish(speed_msg)
                self.ros_node.get_logger().info(f"Published max speed: {self.max_speed}")
            else:
                self.ros_node.get_logger().warn("Speed must be between 0.1 and 10.0 m/s")
        except ValueError:
            self.ros_node.get_logger().warn("Invalid speed value")
    
    def update_ui(self):
        """更新UI显示"""
        self.pos_label.setText(f"Current Position: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f})")
        self.plot_widget.update()

class UINode(Node):
    def __init__(self):
        super().__init__('nav_ui_node')

def main():
    # 初始化ROS2
    rclpy.init()
    
    # 创建ROS2节点
    ros_node = UINode()
    
    # 创建PyQt应用
    app = QApplication(sys.argv)
    ui = NavigationUI(ros_node)
    ui.show()
    
    # 运行ROS2和Qt事件循环
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 100Hz
    
    # 运行应用
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
