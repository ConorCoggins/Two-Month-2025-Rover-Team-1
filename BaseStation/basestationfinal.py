import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QGridLayout, QMessageBox
)
from PyQt6.QtCore import QTimer, pyqtSignal
from threading import Thread
import pyqtgraph as pg

# Try to import and init DualSense
ds = None
try:
    from pydualsense import pydualsense
    ds = pydualsense()
    ds.init()
except Exception as e:
    print(f"DualSense init failed: {e}")

# Global sensor data
temperature = 0.0  # °C
pressure = 0.0     # hPa
acceleration = 0.0 # ft/s²
altitude = 0.0     # ft

class BaseStationTalkerNode(Node):
    def __init__(self):
        super().__init__('base_station_talker')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        self.get_logger().info('Base Station Talker node started')

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')

class BaseStationListenerNode(Node):
    def __init__(self, update_signal):
        super().__init__('base_station_listener')
        self.update_signal = update_signal

        self.sub_acceleration = self.create_subscription(
            Float32, '/acceleration', self.acc_callback, 10)
        self.sub_altitude = self.create_subscription(
            Float32, '/altitude', self.alt_callback, 10)
        self.sub_temperature = self.create_subscription(
            Float32, '/temperature', self.temp_callback, 10)
        self.sub_pressure = self.create_subscription(
            Float32, '/pressure', self.press_callback, 10)

        self.get_logger().info("Base Station Listener Node started, subscribing to topics")

    def acc_callback(self, msg):
        global acceleration
        acceleration = msg.data
        self.update_signal.emit()

    def alt_callback(self, msg):
        global altitude
        altitude = msg.data
        self.update_signal.emit()

    def temp_callback(self, msg):
        global temperature
        temperature = msg.data
        self.update_signal.emit()

    def press_callback(self, msg):
        global pressure
        pressure = msg.data
        self.update_signal.emit()

class Ros2Thread(Thread):
    def __init__(self, update_signal):
        super().__init__()
        self.update_signal = update_signal

    def run(self):
        node = BaseStationListenerNode(self.update_signal)
        rclpy.spin(node)
        node.destroy_node()

class ControlWindow(QWidget):
    def __init__(self, talker_node):
        super().__init__()
        self.node = talker_node
        self.setWindowTitle("Robotic Arm Control (No Controller - Buttons Only)" if not ds else "Robotic Arm Control")
        self.setGeometry(100, 100, 400, 300)

        # Buttons
        btn_style = "min-width: 120px; min-height: 30px;"
        self.open_claw = QPushButton("Open Claw")
        self.close_claw = QPushButton("Close Claw")
        self.wrist_up = QPushButton("Wrist Up")
        self.wrist_down = QPushButton("Wrist Down")
        self.elbow1_up = QPushButton("Elbow 1 Up")
        self.elbow1_down = QPushButton("Elbow 1 Down")
        self.elbow2_up = QPushButton("Elbow 2 Up")
        self.elbow2_down = QPushButton("Elbow 2 Down")
        self.turret_left = QPushButton("Turret Left")
        self.turret_right = QPushButton("Turret Right")

        for b in [self.open_claw, self.close_claw, self.wrist_up, self.wrist_down,
                  self.elbow1_up, self.elbow1_down, self.elbow2_up, self.elbow2_down,
                  self.turret_left, self.turret_right]:
            b.setStyleSheet(btn_style)

        # Command lambdas
        self.open_claw.clicked.connect(lambda: self.send_command("OPENCLAW"))
        self.close_claw.clicked.connect(lambda: self.send_command("CLOSECLAW"))
        self.wrist_up.clicked.connect(lambda: self.send_command("WRISTUP"))
        self.wrist_down.clicked.connect(lambda: self.send_command("WRISTDOWN"))
        self.elbow1_up.clicked.connect(lambda: self.send_command("ELBOW1UP"))
        self.elbow1_down.clicked.connect(lambda: self.send_command("ELBOW1DOWN"))
        self.elbow2_up.clicked.connect(lambda: self.send_command("ELBOW2UP"))
        self.elbow2_down.clicked.connect(lambda: self.send_command("ELBOW2DOWN"))
        self.turret_left.clicked.connect(lambda: self.send_command("TURRETLEFT"))
        self.turret_right.clicked.connect(lambda: self.send_command("TURRETRIGHT"))

        # Layout
        main = QVBoxLayout()
        main.setContentsMargins(10, 10, 10, 10)
        main.setSpacing(10)

        claw_box = QHBoxLayout()
        claw_box.addWidget(self.open_claw)
        claw_box.addWidget(self.close_claw)
        main.addWidget(QLabel("<b>Claw Controls</b>"))
        main.addLayout(claw_box)

        wrist_box = QHBoxLayout()
        wrist_box.addWidget(self.wrist_up)
        wrist_box.addWidget(self.wrist_down)
        main.addWidget(QLabel("<b>Wrist Controls</b>"))
        main.addLayout(wrist_box)

        elbow_box = QHBoxLayout()
        elbow1_box = QVBoxLayout()
        elbow1_box.addWidget(self.elbow1_up)
        elbow1_box.addWidget(self.elbow1_down)
        elbow2_box = QVBoxLayout()
        elbow2_box.addWidget(self.elbow2_up)
        elbow2_box.addWidget(self.elbow2_down)
        elbow_box.addLayout(elbow1_box)
        elbow_box.addLayout(elbow2_box)
        main.addWidget(QLabel("<b>Elbow Controls</b>"))
        main.addLayout(elbow_box)

        turret_box = QHBoxLayout()
        turret_box.addWidget(self.turret_left)
        turret_box.addWidget(self.turret_right)
        main.addWidget(QLabel("<b>Turret Controls</b>"))
        main.addLayout(turret_box)

        self.setLayout(main)

        # Controller polling
        if ds:
            def poll_controller():
                state = ds.sendReport()
                if not state:
                    return

                if state.DPadUp: self.send_command("WRISTUP")
                if state.DPadDown: self.send_command("WRISTDOWN")
                if state.DPadLeft: self.send_command("TURRETLEFT")
                if state.DPadRight: self.send_command("TURRETRIGHT")
                if state.L2 > 128: self.send_command("OPENCLAW")
                if state.R2 > 128: self.send_command("CLOSECLAW")
                if state.LeftStickY > 0.5: self.send_command("ELBOW1UP")
                if state.LeftStickY < -0.5: self.send_command("ELBOW1DOWN")
                if state.RightStickY > 0.5: self.send_command("ELBOW2UP")
                if state.RightStickY < -0.5: self.send_command("ELBOW2DOWN")
                if any([state.DPadUp, state.DPadDown, state.DPadLeft, state.DPadRight,
                        state.L2 > 128, state.R2 > 128, abs(state.LeftStickY) > 0.5, abs(state.RightStickY) > 0.5]):
                    self.trigger_feedback()

            controller_timer = QTimer(self)
            controller_timer.timeout.connect(poll_controller)
            controller_timer.start(50)  # Poll at 20 Hz

    def send_command(self, command):
        try:
            self.node.send_command(command)
            QMessageBox.information(self, "Command Sent", f"Sent command: {command}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to send command: {e}")

    def trigger_feedback(self):
        if ds:
            ds.trigger.set_mode(Mode.Vibration)
            ds.trigger.set_effect(0.5, 0.1)  # Light rumble

class PlotWindow(QWidget):
    update_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Graphical Information")
        self.setGeometry(520, 100, 800, 600)

        # Widgets
        self.temp_label = QLabel("Temperature: –")
        self.temp_graph = pg.PlotWidget(title="Temperature")
        self.temp_graph.setLabel('left', 'Temperature (°C)')
        self.temp_graph.setLabel('bottom', 'Time (s)')
        self.temp_curve = self.temp_graph.plot(pen='b', symbol='o', symbolPen='b', symbolBrush='b')

        self.pres_label = QLabel("Pressure: –")
        self.pres_graph = pg.PlotWidget(title="Pressure")
        self.pres_graph.setLabel('left', 'Pressure (hPa)')
        self.pres_graph.setLabel('bottom', 'Time (s)')
        self.pres_curve = self.pres_graph.plot(pen='g', symbol='o', symbolPen='g', symbolBrush='g')

        self.acc_label = QLabel("Acceleration: –")
        self.acc_graph = pg.PlotWidget(title="Acceleration")
        self.acc_graph.setLabel('left', 'Acceleration (ft/s²)')
        self.acc_graph.setLabel('bottom', 'Time (s)')
        self.acc_curve = self.acc_graph.plot(pen='r', symbol='o', symbolPen='r', symbolBrush='r')

        self.alt_label = QLabel("Altitude: –")
        self.alt_graph = pg.PlotWidget(title="Altitude")
        self.alt_graph.setLabel('left', 'Altitude (ft)')
        self.alt_graph.setLabel('bottom', 'Time (s)')
        self.alt_curve = self.alt_graph.plot(pen='y', symbol='o', symbolPen='y', symbolBrush='y')

        # Data buffers
        self.max_points = 60
        self.t_data = []
        self.temp_data, self.pres_data, self.acc_data, self.alt_data = [], [], [], []
        self.start_time = time.time()

        # Grid layout
        layout = QGridLayout()
        layout.addWidget(self.temp_label, 0, 0)
        layout.addWidget(self.temp_graph, 1, 0)
        layout.addWidget(self.pres_label, 0, 1)
        layout.addWidget(self.pres_graph, 1, 1)
        layout.addWidget(self.acc_label, 2, 0)
        layout.addWidget(self.acc_graph, 3, 0)
        layout.addWidget(self.alt_label, 2, 1)
        layout.addWidget(self.alt_graph, 3, 1)
        self.setLayout(layout)

        # Connect update signal
        self.update_signal.connect(self.update)

        # Timer for periodic updates
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1000)  # Update every second

    def update(self):
        now = time.time() - self.start_time

        # Update labels
        self.temp_label.setText(f"Temperature: {temperature:.2f} °C")
        self.pres_label.setText(f"Pressure: {pressure:.2f} hPa")
        self.acc_label.setText(f"Acceleration: {acceleration:.2f} ft/s²")
        self.alt_label.setText(f"Altitude: {altitude:.2f} ft")

        # Append data
        self.t_data.append(now)
        self.temp_data.append(temperature)
        self.pres_data.append(pressure)
        self.acc_data.append(acceleration)
        self.alt_data.append(altitude)

        # Trim data if necessary
        if len(self.t_data) > self.max_points:
            self.t_data = self.t_data[-self.max_points:]
            self.temp_data = self.temp_data[-self.max_points:]
            self.pres_data = self.pres_data[-self.max_points:]
            self.acc_data = self.acc_data[-self.max_points:]
            self.alt_data = self.alt_data[-self.max_points:]

        # Update curves
        self.temp_curve.setData(self.t_data, self.temp_data)
        self.pres_curve.setData(self.t_data, self.pres_data)
        self.acc_curve.setData(self.t_data, self.acc_data)
        self.alt_curve.setData(self.t_data, self.alt_data)

def no_controller_window():
    app = QApplication(sys.argv) if QApplication.instance() is None else QApplication.instance()
    win = QWidget()
    win.setWindowTitle("Alert")
    win.setGeometry(300, 300, 400, 150)

    label = QLabel("There is currently no controller detected. Check if device is on and connected to PC.")
    label.setWordWrap(True)
    confirm = QPushButton("Ok")
    confirm.clicked.connect(app.quit)

    layout = QVBoxLayout()
    layout.addWidget(label)
    layout.addWidget(confirm)
    win.setLayout(layout)
    win.show()
    sys.exit(app.exec())

def main():
    rclpy.init()
    talker_node = BaseStationTalkerNode()
    app = QApplication(sys.argv)

    if not ds:
        no_controller_window()

    plot_win = PlotWindow()
    ctrl_win = ControlWindow(talker_node)
    ctrl_win.show()
    plot_win.show()

    # Start ROS 2 thread
    ros_thread = Ros2Thread(plot_win.update_signal)
    ros_thread.start()

    # Cleanup
    def cleanup():
        if ds:
            ds.close()
        rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec())

if __name__ == "__main__":
    main()