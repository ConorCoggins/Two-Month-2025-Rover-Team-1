import sys
import time
import random
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton
)
from PyQt6.QtCore import QTimer
import pyqtgraph as pg

# ----------------------------------------------------------------------
# 1. Global sensor placeholders (replace with real data acquisition later)
# ----------------------------------------------------------------------
temperature = None      # °C
pressure    = None      # hPa
acceleration = None     # ft/s²
altitude    = None      # ft

# ----------------------------------------------------------------------
# 2. Control window (unchanged except for minor cosmetic tweaks)
# ----------------------------------------------------------------------
def create_control_window() -> QWidget:
    win = QWidget()
    win.setWindowTitle("Robotic Arm Control")
    win.setGeometry(100, 100, 400, 300)

    # -- buttons -------------------------------------------------------
    btn_style = "min-width: 120px; min-height: 30px;"

    open_claw = QPushButton("Open Claw")
    close_claw = QPushButton("Close Claw")
    wrist_up = QPushButton("Wrist Up")
    wrist_down = QPushButton("Wrist Down")
    elbow1_up = QPushButton("Elbow 1 Up")
    elbow1_down = QPushButton("Elbow 1 Down")
    elbow2_up = QPushButton("Elbow 2 Up")
    elbow2_down = QPushButton("Elbow 2 Down")
    turret_left = QPushButton("Turret Left")
    turret_right = QPushButton("Turret Right")

    for b in [open_claw, close_claw, wrist_up, wrist_down,
              elbow1_up, elbow1_down, elbow2_up, elbow2_down,
              turret_left, turret_right]:
        b.setStyleSheet(btn_style)

    # -- connections (just prints for now) ----------------------------
    open_claw.clicked.connect(lambda: print("Claw opened"))
    close_claw.clicked.connect(lambda: print("Claw closed"))
    wrist_up.clicked.connect(lambda: print("Wrist moved up"))
    wrist_down.clicked.connect(lambda: print("Wrist moved down"))
    elbow1_up.clicked.connect(lambda: print("Elbow 1 moved up"))
    elbow1_down.clicked.connect(lambda: print("Elbow 1 moved down"))
    elbow2_up.clicked.connect(lambda: print("Elbow 2 moved up"))
    elbow2_down.clicked.connect(lambda: print("Elbow 2 moved down"))
    turret_left.clicked.connect(lambda: print("Turret moved left"))
    turret_right.clicked.connect(lambda: print("Turret moved right"))

    # -- layout --------------------------------------------------------
    main = QVBoxLayout()
    main.setContentsMargins(10, 10, 10, 10)
    main.setSpacing(10)

    # Claw
    claw_box = QHBoxLayout()
    claw_box.addWidget(open_claw)
    claw_box.addWidget(close_claw)
    main.addWidget(QLabel("<b>Claw Controls</b>"))
    main.addLayout(claw_box)

    # Wrist
    wrist_box = QHBoxLayout()
    wrist_box.addWidget(wrist_up)
    wrist_box.addWidget(wrist_down)
    main.addWidget(QLabel("<b>Wrist Controls</b>"))
    main.addLayout(wrist_box)

    # Elbow
    elbow_box = QHBoxLayout()
    elbow1_box = QVBoxLayout()
    elbow1_box.addWidget(elbow1_up)
    elbow1_box.addWidget(elbow1_down)
    elbow2_box = QVBoxLayout()
    elbow2_box.addWidget(elbow2_up)
    elbow2_box.addWidget(elbow2_down)
    elbow_box.addLayout(elbow1_box)
    elbow_box.addLayout(elbow2_box)
    main.addWidget(QLabel("<b>Elbow Controls</b>"))
    main.addLayout(elbow_box)

    # Turret
    turret_box = QHBoxLayout()
    turret_box.addWidget(turret_left)
    turret_box.addWidget(turret_right)
    main.addWidget(QLabel("<b>Turret Controls</b>"))
    main.addLayout(turret_box)

    win.setLayout(main)
    return win


# ----------------------------------------------------------------------
# 3. Plot window – now uses a QTimer and proper data containers
# ----------------------------------------------------------------------
def create_plot_window() -> QWidget:
    win = QWidget()
    win.setWindowTitle("Graphical Information")
    win.setGeometry(520, 100, 800, 600)

    # ------------------------------------------------------------------
    # 3.1 Widgets for each sensor (label + graph)
    # ------------------------------------------------------------------
    temp_label = QLabel("Temperature: –")
    temp_graph = pg.PlotWidget(title="Temperature")
    temp_graph.setLabel('left', 'Temperature (°C)')
    temp_graph.setLabel('bottom', 'Time (s)')
    temp_curve = temp_graph.plot(pen='b', symbol='o', symbolPen='b', symbolBrush='b')

    pres_label = QLabel("Pressure: –")
    pres_graph = pg.PlotWidget(title="Pressure")
    pres_graph.setLabel('left', 'Pressure (hPa)')
    pres_graph.setLabel('bottom', 'Time (s)')
    pres_curve = pres_graph.plot(pen='g', symbol='o', symbolPen='g', symbolBrush='g')

    acc_label = QLabel("Acceleration: –")
    acc_graph = pg.PlotWidget(title="Acceleration")
    acc_graph.setLabel('left', 'Acceleration (ft/s²)')
    acc_graph.setLabel('bottom', 'Time (s)')
    acc_curve = acc_graph.plot(pen='r', symbol='o', symbolPen='r', symbolBrush='r')

    alt_label = QLabel("Altitude: –")
    alt_graph = pg.PlotWidget(title="Altitude")
    alt_graph.setLabel('left', 'Altitude (ft)')
    alt_graph.setLabel('bottom', 'Time (s:')
    alt_curve = alt_graph.plot(pen='y', symbol='o', symbolPen='y', symbolBrush='y')

    # ------------------------------------------------------------------
    # 3.2 Data buffers (kept short for demo; grow as needed)
    # ------------------------------------------------------------------
    max_points = 60                     # last minute if updated @ 1 Hz
    t_data = []                         # time stamps
    temp_data, pres_data, acc_data, alt_data = [], [], [], []

    start_time = time.time()

    # ------------------------------------------------------------------
    # 3.3 Update logic – runs every 1000 ms
    # ------------------------------------------------------------------
    def update():
        nonlocal t_data, temp_data, pres_data, acc_data, alt_data

        now = time.time() - start_time

        # ---- Simulate incoming telemetry (replace with real reads) ----
        global temperature, pressure, acceleration, altitude
        temperature = random.uniform(15, 35) if temperature is None else temperature + random.uniform(-0.5, 0.5)
        pressure    = random.uniform(900, 1100) if pressure is None else pressure + random.uniform(-2, 2)
        acceleration = random.uniform(-5, 5) if acceleration is None else acceleration + random.uniform(-0.1, 0.1)
        altitude    = random.uniform(0, 5000) if altitude is None else altitude + random.uniform(-10, 10)

        # ---- Update labels ------------------------------------------------
        temp_label.setText(f"Temperature: {temperature:.2f} °C" if temperature is not None else "Temperature: No data")
        pres_label.setText(f"Pressure: {pressure:.2f} hPa" if pressure is not None else "Pressure: No data")
        acc_label.setText(f"Acceleration: {acceleration:.2f} ft/s²" if acceleration is not None else "Acceleration: No data")
        alt_label.setText(f"Altitude: {altitude:.2f} ft" if altitude is not None else "Altitude: No data")

        # ---- Append to buffers --------------------------------------------
        t_data.append(now)
        temp_data.append(temperature)
        pres_data.append(pressure)
        acc_data.append(acceleration)
        alt_data.append(altitude)

        # Keep only the last `max_points` entries
        if len(t_data) > max_points:
            t_data = t_data[-max_points:]
            temp_data = temp_data[-max_points:]
            pres_data = pres_data[-max_points:]
            acc_data = acc_data[-max_points:]
            alt_data = alt_data[-max_points:]

        # ---- Push new data to curves ---------------------------------------
        temp_curve.setData(t_data, temp_data)
        pres_curve.setData(t_data, pres_data)
        acc_curve.setData(t_data, acc_data)
        alt_curve.setData(t_data, alt_data)

    # ------------------------------------------------------------------
    # 3.4 Layout assembly
    # ------------------------------------------------------------------
    top = QHBoxLayout()
    top.addWidget(temp_label)
    top.addWidget(temp_graph)
    top.addWidget(pres_label)
    top.addWidget(pres_graph)

    bottom = QHBoxLayout()
    bottom.addWidget(acc_label)
    bottom.addWidget(acc_graph)
    bottom.addWidget(alt_label)
    bottom.addWidget(alt_graph)

    main_layout = QVBoxLayout()
    main_layout.addLayout(top)
    main_layout.addLayout(bottom)
    win.setLayout(main_layout)

    # ------------------------------------------------------------------
    # 3.5 Start the timer
    # ------------------------------------------------------------------
    timer = QTimer(win)
    timer.timeout.connect(update)
    timer.start(1000)      # 1 Hz updates

    return win


# ----------------------------------------------------------------------
# 4. Application entry point
# ----------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)

    ctrl_win = create_control_window()
    plot_win = create_plot_window()

    ctrl_win.show()
    plot_win.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()