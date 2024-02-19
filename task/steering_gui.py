import sys
import math
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QFont,QColor
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pacmod3_msgs.msg import SteeringCmd, VehicleSpeedRpt, SteeringAuxRpt

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.init_subscriptions()

        self.last_driving_status = None
        self.last_reference_speed = None
        self.last_current_speed = None
        self.last_steering_angle = None
        self.last_current_steering_angle = None

        # Initialize PyQt application
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("ROS2 GUI")
        self.window.setGeometry(200, 200, 600, 600)

        # Create QLabel instances for displaying data
        self.label_status = QLabel(self.window)
        self.label_status.setGeometry(200, 20, 600, 100)
        self.label_status.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.label_status.setFont(QFont("Arial", 18))

        self.label_ref_speed = QLabel(self.window)
        self.label_ref_speed.setGeometry(10, 150, 600, 30)
        self.label_ref_speed.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.label_ref_speed.setFont(QFont("Arial", 14),)

        self.label_current_speed = QLabel(self.window)
        self.label_current_speed.setGeometry(10, 100, 600, 30)
        self.label_current_speed.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.label_current_speed.setFont(QFont("Arial", 14))

        self.label_current_angle = QLabel(self.window)
        self.label_current_angle.setGeometry(10, 130, 400, 30)
        self.label_current_angle.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.label_current_angle.setFont(QFont("Arial", 12))

        self.label_ref_angle = QLabel(self.window)
        self.label_ref_angle.setGeometry(10, 170, 400, 30)
        self.label_ref_angle.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.label_ref_angle.setFont(QFont("Arial", 12))

        self.window.show()

    def init_subscriptions(self):
        self.subscription1 = self.create_subscription(Bool, '/lexus3/pacmod/enabled', self.listener_callback1, 1)
        self.subscription2 = self.create_subscription(Twist, '/lexus3/cmd_vel', self.listener_callback2, 1)
        self.subscription3 = self.create_subscription(VehicleSpeedRpt, '/lexus3/pacmod/vehicle_speed_rpt', self.listener_callback3, 1)
        self.subscription4 = self.create_subscription(SteeringAuxRpt, '/lexus3/pacmod/steering_aux_rpt', self.listener_callback4, 1)
        self.subscription5 = self.create_subscription(SteeringCmd, '/lexus3/pacmod/steering_cmd', self.listener_callback5, 1)

    def update_display(self):
        if self.last_driving_status is not None:
            self.label_status.setText(self.last_driving_status)

        if self.last_reference_speed is not None:
            self.label_ref_speed.setText("Reference Speed: {:.2f} Km/h".format(self.last_reference_speed))

        if self.last_current_speed is not None:
            self.label_current_speed.setText("Current Speed: {:z.2f} Km/h".format(self.last_current_speed))

        if self.last_current_steering_angle is not None:
            self.label_current_angle.setText("Current angle: {:.2f}°".format(self.last_current_steering_angle))

        if self.last_steering_angle is not None:
            self.label_ref_angle.setText("Ref angle: {:.2f}°".format(self.last_steering_angle))

    def listener_callback1(self, msg):
        print("Received message on /lexus3/pacmod/enabled topic:", msg)
        self.last_driving_status = 'You Are Driving' if msg.data else 'In Car Driver'
        self.update_display()

    def listener_callback2(self, msg):
        print("Received message on /lexus3/cmd_vel topic:", msg)
        self.last_reference_speed = msg.linear.x
        self.update_display()

    def listener_callback3(self, msg):
        print("Received message on /lexus3/pacmod/vehicle_speed_rpt topic:", msg)
        self.last_current_speed = msg.vehicle_speed
        self.update_display()

    def listener_callback4(self, msg):
        print("Received message on /lexus3/pacmod/steering_aux_rpt topic:", msg)
        self.last_current_steering_angle = math.degrees(msg.rotation_rate)
        self.update_display()

    def listener_callback5(self, msg):
        print("Received message on /lexus3/pacmod/steering_cmd topic:", msg)
        self.last_steering_angle = math.degrees(msg.rotation_rate)
        self.update_display()

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    try:
        while rclpy.ok():
            rclpy.spin_once(minimal_subscriber)
            minimal_subscriber.app.processEvents()
    except KeyboardInterrupt:
        pass

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(minimal_subscriber.app.exec_())

if __name__ == '__main__':
    main()
