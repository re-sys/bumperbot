from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLineEdit, QPushButton
from std_msgs.msg import Float64MultiArray  # 或 geometry_msgs/Point

class CoordinatePublisher(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._widget = QWidget()
        self._layout = QVBoxLayout()

        # 创建输入框和按钮
        self._x_input = QLineEdit("0.0")
        self._y_input = QLineEdit("0.0")
        self._publish_button = QPushButton("Publish Coordinates")

        # 添加到布局
        self._layout.addWidget(self._x_input)
        self._layout.addWidget(self._y_input)
        self._layout.addWidget(self._publish_button)
        self._widget.setLayout(self._layout)

        # 设置窗口标题
        self._widget.setWindowTitle("Coordinate Publisher")

        # 添加到 rqt 界面
        context.add_widget(self._widget)

        # 创建发布者
        self._publisher = self._node.create_publisher(
            Float64MultiArray,  # 或 Point/Pose 消息类型
            "/target_coordinates",
            10
        )

        # 连接按钮信号
        self._publish_button.clicked.connect(self._publish_coordinates)

    def _publish_coordinates(self):
        try:
            x = float(self._x_input.text())
            y = float(self._y_input.text())
            msg = Float64MultiArray(data=[x, y])  # 构造消息
            self._publisher.publish(msg)
            self._node.get_logger().info(f"Published: [{x}, {y}]")
        except ValueError:
            self._node.get_logger().error("Invalid input! Must be numbers.")
