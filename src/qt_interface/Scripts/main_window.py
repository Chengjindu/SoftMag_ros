#!/usr/bin/env python3

import sys
import signal
import json
import rospy
from std_srvs.srv import Trigger
import numpy as np
from std_msgs.msg import String, Int32, Float32
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QSizePolicy, QCheckBox
from PySide6.QtCore import  (Signal, Slot, QTimer, QThread, QMutex, QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect, QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor, QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter, QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtOpenGLWidgets import QOpenGLWidget
import subprocess
import seaborn as sns
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ui_mainwindow import Ui_MainWindow


np.random.BitGenerator = np.random.bit_generator

sns.set_theme(context='notebook', style='whitegrid', palette='muted')   # Set the Seaborn style

steps_per_rev = 200
full_stroke_angle = 85
max_steps = int((full_stroke_angle / 360) * steps_per_rev)

max_pressure = 50
pressure_ctrl_resolution = 0.2


class MatplotlibCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MatplotlibCanvas, self).__init__(self.fig)
        self.setParent(parent)

    def plot(self, x, y, label, color):
        self.axes.plot(x, y, label=label, color=color)
        self.axes.legend()
        self.draw()

    def clear_plot(self):
        self.axes.cla()
        self.axes.grid(True)
        self.draw()

    def set_labels(self, xlabel, ylabel):
        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)

    def set_xlim(self, left, right):
        self.axes.set_xlim(left, right)

    def set_ylim(self, bottom, top):
        self.axes.set_ylim(bottom, top)

    def enable_grid(self):
        self.axes.grid(True)


# Create a Matplotlib canvas class that can update its plot dynamically
class SensorDataCanvas(FigureCanvas):
    def __init__(self, parent=None, window_size=30, sensor_title=''):
        # Apply the Seaborn style
        # sns.set()
        plt.style.use('ggplot')  # Choose a style for the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        super(SensorDataCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.window_size = window_size
        self.x_data, self.y_data, self.z_data = [], [], []
        self.setup_plot(sensor_title)

    def setup_plot(self, sensor_title):
        self.fig.subplots_adjust(left=0.15, right=0.9, top=0.9, bottom=0.15)
        self.ax.set_title(sensor_title, fontsize=10)
        # self.ax.set_xlabel('Time', fontsize=8)
        # self.ax.set_ylabel('Sensor Reading', fontsize=8)
        self.ax.grid(True)
        # Initialize empty lines for each axis and store them
        self.lines = {
            'x': self.ax.plot(self.x_data, label='X-Axis', color='r')[0],
            'y': self.ax.plot(self.y_data, label='Y-Axis', color='g')[0],
            'z': self.ax.plot(self.z_data, label='Z-Axis', color='b')[0]
        }
        self.ax.legend(loc='upper right', handlelength=2, handletextpad=0.5, borderaxespad=0.5, fontsize=8)
        self.ax.annotate('(s)  ', xy=(1.05, -0.05), xycoords='axes fraction', ha='left', va='center')
        self.ax.annotate('(G)  ', xy=(-0.05, 1.02), xycoords='axes fraction', ha='center', va='bottom', rotation=0)


    def update_plot(self, x, y, z):
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        # Update the data of each line
        self.lines['x'].set_data(range(len(self.x_data)), self.x_data)
        self.lines['y'].set_data(range(len(self.y_data)), self.y_data)
        self.lines['z'].set_data(range(len(self.z_data)), self.z_data)

        self.rescale_axes()
        # self.draw()  # Redraw the canvas

    def rescale_axes(self):
        # Rescale x-axis to show a rolling window
        sample_number = len(self.x_data)
        self.ax.set_xlim(max(0, sample_number - self.window_size), max(self.window_size, sample_number))

        # Dynamic y-axis scaling based on the visible data
        all_visible_data = self.x_data[-self.window_size:] + self.y_data[-self.window_size:] + self.z_data[-self.window_size:]
        y_min, y_max = min(all_visible_data), max(all_visible_data)
        self.ax.set_ylim(y_min - 0.1, y_max + 0.1)  # Add some padding

        # Redraw the canvas after the update
        self.draw()

    def clear_plot(self):
        self.x_data, self.y_data, self.z_data = [], [], []
        self.lines['x'].set_data([], [])
        self.lines['y'].set_data([], [])
        self.lines['z'].set_data([], [])
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(-3, 3)
        self.draw()

class QuadrantCanvas(FigureCanvas):
    def __init__(self, parent=None):
        fig = Figure()  # Define the figure size
        self.ax = fig.add_subplot(111, aspect='equal')
        self.ax.set_xlim(0, 1)  # Set the x-axis limits to be from 0 to 1
        self.ax.set_ylim(0, 1)  # Set the y-axis limits to be from 0 to 1
        self.ax.axis('off')    # Hide the axes

        # Define colors
        self.default_color = 'lightgrey'
        self.active_color = 'blue'

        self.quadrants = [# Initialize the quadrant patches
            patches.Rectangle((0, 0), 0.5, 0.5, facecolor=self.default_color),
            patches.Rectangle((0, 0.5), 0.5, 0.5, facecolor=self.default_color),
            patches.Rectangle((0.5, 0.5), 0.5, 0.5, facecolor=self.default_color),
            patches.Rectangle((0.5, 0), 0.5, 0.5, facecolor=self.default_color)
        ]

        # Add quadrants to the axis
        for quad in self.quadrants:
            self.ax.add_patch(quad)

        super(QuadrantCanvas, self).__init__(fig)
        self.setParent(parent)
    #     fig.subplots_adjust(left=0, right=1, top=1, bottom=0)  # Adjust subplot params.
    #
    # def sizeHint(self):
    #     return QSize(100, 100)  # Suggest a default size (will be square due to heightForWidth).
    #
    # def heightForWidth(self, width):
    #     return width  # Maintain a square aspect ratio.

    def update_quadrant(self, sensor_id, prediction, force_value, touch_mode="single"):
        # Reset all quadrants to the default color
        for quad in self.quadrants:
            quad.set_facecolor(self.default_color)

        # Update quadrants based on the force_value and prediction
        if force_value >= 0.5:  # Apply threshold to check for significant force
            if prediction is not None:
                # Single touch mode
                if touch_mode == "single" and 1 <= prediction <= 4:
                    self.quadrants[prediction - 1].set_facecolor(self.active_color)

                # Multi-touch mode
                elif touch_mode == "multi":
                    for i, touch in enumerate(prediction):
                        if touch == 1:  # If touch is detected in the quadrant
                            self.quadrants[i].set_facecolor(self.active_color)

        # Redraw the canvas to show updated colors
        self.draw()

class ForceDataCanvas(FigureCanvas):
    def __init__(self, parent=None, window_size=30, title=''):
        plt.style.use('ggplot')
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        super(ForceDataCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.window_size = window_size
        self.force_data_s1 = []
        self.force_data_s2 = []
        self.setup_plot(title)

    def setup_plot(self, title):
        self.fig.subplots_adjust(left=0.15, right=0.9, top=0.9, bottom=0.15)
        self.ax.set_title(title, fontsize=10)
        self.ax.grid(True)
        self.line_s1, = self.ax.plot(self.force_data_s1, label='Force S1', color='m')
        self.line_s2, = self.ax.plot(self.force_data_s2, label='Force S2', color='c')
        self.ax.legend(loc='upper right', handlelength=2, handletextpad=0.5, borderaxespad=0.5, fontsize=8)
        self.ax.annotate(' (s)  ', xy=(1.05, -0.05), xycoords='axes fraction', ha='left', va='center')
        self.ax.annotate(' (N)  ', xy=(-0.05, 1.02), xycoords='axes fraction', ha='center', va='bottom', rotation=0)

    def update_plot(self, sensor_id, force):
        if sensor_id == 'S1':
            self.force_data_s1.append(force)
            self.line_s1.set_data(range(len(self.force_data_s1)), self.force_data_s1)
        elif sensor_id == 'S2':
            self.force_data_s2.append(force)
            self.line_s2.set_data(range(len(self.force_data_s2)), self.force_data_s2)
        self.rescale_axes()

    def rescale_axes(self):
        sample_number = max(len(self.force_data_s1), len(self.force_data_s2))
        self.ax.set_xlim(max(0, sample_number - self.window_size), max(self.window_size, sample_number))
        all_force_data = self.force_data_s1[-self.window_size:] + self.force_data_s2[-self.window_size:]
        y_min, y_max = min(all_force_data), max(all_force_data)
        self.ax.set_ylim(y_min - 0.1, y_max + 0.1)
        self.draw()

    def clear_plot(self):
        self.force_data_s1 = []
        self.force_data_s2 = []
        self.line_s1.set_data([], [])
        self.line_s2.set_data([], [])
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(0, 3)
        self.draw()

class PressureDataCanvas(FigureCanvas):
    def __init__(self, parent=None, window_size=30):
        plt.style.use('ggplot')  # Consistent style with the sensor plot
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        super(PressureDataCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.window_size = window_size
        self.pressure_data = []
        self.setup_plot()

    def setup_plot(self):
        self.fig.subplots_adjust(left=0.15, right=0.9, top=0.9, bottom=0.15)
        self.ax.set_title('Pressure Monitoring', fontsize=10)
        # self.ax.set_xlabel('Time', fontsize=10)
        # self.ax.set_ylabel('Pressure (kPa)', fontsize=10)
        self.ax.grid(True)
        # Initialize an empty line for the pressure data
        self.pressure_line, = self.ax.plot([], [], label='Pressure', color='b')
        self.ax.legend(loc='upper right', handlelength=2, handletextpad=0.5, borderaxespad=0.5, fontsize=8)
        self.ax.annotate(' (s)  ', xy=(1.05, -0.05), xycoords='axes fraction', ha='left', va='center')
        self.ax.annotate(' (kPa)  ', xy=(-0.05, 1.02), xycoords='axes fraction', ha='center', va='bottom', rotation=0)

    def update_plot(self, pressure_value):
        self.pressure_data.append(pressure_value)

        # Update the data of the line
        self.pressure_line.set_data(range(len(self.pressure_data)), self.pressure_data)

        self.rescale_axes()
        self.draw()  # Redraw the canvas

    def rescale_axes(self):
        # Rescale x-axis to show a rolling window
        sample_number = len(self.pressure_data)
        self.ax.set_xlim(max(0, sample_number - self.window_size), max(self.window_size, sample_number))

        # Dynamic y-axis scaling based on the visible data
        visible_data = self.pressure_data[-self.window_size:]
        y_min, y_max = min(visible_data), max(visible_data)
        self.ax.set_ylim(y_min - 1, y_max + 1)  # Add some padding

    def clear_plot(self):
        self.pressure_data = []
        self.pressure_line.set_data([], [])
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(0, 100)  # Assuming pressure range from 0 to 100 kPa
        self.draw()

class ModeSwitchWorker(QThread):
    mode_switched = Signal(str, bool)

    def __init__(self, mode, parent=None):
        super(ModeSwitchWorker, self).__init__(parent)
        self.mode = mode

    def run(self):
        success = False
        if self.mode == "Sensing":
            success = self.switch_to_sensing()
        elif self.mode == "Automatic":
            success = self.switch_to_automatic()
        elif self.mode == "Testing":
            success = self.switch_to_testing()
        self.mode_switched.emit(self.mode, success)

    def switch_to_sensing(self):
        rospy.wait_for_service('/switch_to_sensing')
        try:
            switch_service = rospy.ServiceProxy('/switch_to_sensing', Trigger)
            response = switch_service()
            if response.success:
                rospy.loginfo("Switched to sensing mode")
                return True
            else:
                rospy.logerr("Failed to switch to sensing mode: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return False

    def switch_to_automatic(self):
        rospy.wait_for_service('/switch_to_automatic')
        try:
            switch_service = rospy.ServiceProxy('/switch_to_automatic', Trigger)
            response = switch_service()
            if response.success:
                rospy.loginfo("Switched to automatic mode")
                return True
            else:
                rospy.logerr("Failed to switch to automatic mode: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return False

    def switch_to_testing(self):
        rospy.wait_for_service('/switch_to_testing')
        try:
            switch_service = rospy.ServiceProxy('/switch_to_testing', Trigger)
            response = switch_service()
            if response.success:
                rospy.loginfo("Switched to testing mode")
                return True
            else:
                rospy.logerr("Failed to switch to testing mode: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return False


class MainWindow(QMainWindow):
    sensor_data_signal = Signal(str)
    prediction_results_signal = Signal(str)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.mode_switch_worker = None
        self.current_mode = None  # Add this line to track the current mode

        # Initialize comboBox for model selection
        self.ui.model_selection_comboBox.addItem("SoftMag")
        self.ui.model_selection_comboBox.addItem("Gripper(Automatic)")
        self.ui.model_selection_comboBox.addItem("Gripper(Testing)")
        self.ui.model_selection_comboBox.currentIndexChanged.connect(self.switch_mode)

        # Signal handling for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

        # Create instances for the two sensors plot
        self.sensor_plots = {
            "S1": SensorDataCanvas(self.ui.S1_graph, window_size=30, sensor_title='S1 Real-time Reading'),
            "S2": SensorDataCanvas(self.ui.S2_graph, window_size=30, sensor_title='S2 Real-time Reading')
        }
        # Add the matplotlib canvas to the layout
        self.ui.S1_graph_layout = QVBoxLayout(self.ui.S1_graph)
        self.ui.S2_graph_layout = QVBoxLayout(self.ui.S2_graph)
        self.ui.S1_graph_layout.addWidget(self.sensor_plots["S1"])
        self.ui.S2_graph_layout.addWidget(self.sensor_plots["S2"])

        # Create instances for the two quadrant plot
        self.quadrant_canvas_s1 = QuadrantCanvas(self.ui.quadrant_1_1)
        self.ui.quadrant_1_1_layout = QVBoxLayout(self.ui.quadrant_1_1)
        self.ui.quadrant_1_1_layout.addWidget(self.quadrant_canvas_s1)
        self.quadrant_canvas_s2 = QuadrantCanvas(self.ui.quadrant_2_2)
        self.ui.quadrant_2_2_layout = QVBoxLayout(self.ui.quadrant_2_2)
        self.ui.quadrant_2_2_layout.addWidget(self.quadrant_canvas_s2)

        # Create an instance of the ForceDataCanvas for plotting force values
        self.force_plot = ForceDataCanvas(self.ui.force_val_graph, window_size=30, title='Real-time Force')
        self.ui.force_val_graph_layout = QVBoxLayout(self.ui.force_val_graph)
        self.ui.force_val_graph_layout.addWidget(self.force_plot)

        # Create instances for the two quadrant plot
        self.pressure_plot = PressureDataCanvas(self.ui.pressure_graph, window_size=30)
        self.ui.pressure_graph_layout = QVBoxLayout(self.ui.pressure_graph)
        self.ui.pressure_graph_layout.addWidget(self.pressure_plot)

        # Create instances for the motor position reading bar
        self.ui.motor_pos_reading_bar_1.setRange(0, max_steps)
        self.ui.motor_pos_reading_bar_2.setRange(0, max_steps)

        # Connect the existing checkboxes to the appropriate slots
        self.ui.forceControlCheckBox = self.findChild(QCheckBox, "force_feedback_ctrl_enable")
        self.ui.forceControlCheckBox.stateChanged.connect(self.onForceControlStateChanged)

        self.ui.contactDetectCheckBox = self.findChild(QCheckBox, "contact_detect_enable")
        self.ui.contactDetectCheckBox.stateChanged.connect(self.onContactDetectStateChanged)

        self.ui.sofaSimulationCheckBox = self.findChild(QCheckBox, "sofa_simulation_enable")
        self.ui.sofaSimulationCheckBox.stateChanged.connect(self.onSofaSimulationStateChanged)

        # Set up ROS subscribers
        rospy.Subscriber("/processed_sensor_data", String, self.update_sensor_data)
        rospy.Subscriber("/prediction_results", String, self.update_prediction_results)
        rospy.Subscriber("/pressure_reading", Float32, self.update_pressure_reading)
        rospy.Subscriber("/motor_pos_reading", Int32, self.update_motor_pos_reading)
        rospy.Subscriber("/contact_detect", String, self.contact_detect_callback)
        rospy.Subscriber("/grasp_stable", String, self.grasp_stable_callback)
        rospy.Subscriber("/motor_stop", String, self.motor_stop_callback)

        # Set up ROS publishers for the mode selection
        self.ctrl_mode_publisher = rospy.Publisher('/ctrl_mode', String, queue_size=10)

        # Set up ROS publishers for automatic grasping mode
        self.release_publisher = rospy.Publisher('/release', String, queue_size=10)
        self.restart_publisher = rospy.Publisher('/restart', String, queue_size=10)

        # Set up ROS publishers for testing mode
        self.motor_pos_ctrl_publisher = rospy.Publisher('/motor_pos_ctrl', Int32, queue_size=10)
        self.pressure_ctrl_publisher = rospy.Publisher('/pressure_ctrl', Float32, queue_size=10)
        self.force_closure_publisher = rospy.Publisher('/force_closure', String, queue_size=10)
        self.zero_motor_publisher = rospy.Publisher('/zero_motor', String, queue_size=10)
        self.record_data_publisher = rospy.Publisher('/record_data', String, queue_size=10)

        # Set up stop publishers for both modes
        self.stop_all_publisher = rospy.Publisher('/stop_all', String, queue_size=10)

        # Connect signals
        self.sensor_data_signal.connect(self.on_sensor_data_received)
        self.prediction_results_signal.connect(self.on_prediction_results_received)

        self.ui.release_button.clicked.connect(self.release_callback)
        self.ui.restart_button.clicked.connect(self.restart_callback)
        self.ui.stop_all_button_grasping.clicked.connect(self.stop_all_callback)

        self.ui.record_data_button.clicked.connect(self.toggle_recording)
        self.ui.zero_button_testing.clicked.connect(self.zero_motor_callback)
        self.ui.stop_all_button_testing.clicked.connect(self.stop_all_callback)
        self.ui.motor_pos_ctrl_bar_1.valueChanged.connect(self.update_motor_pos_ctrl)
        self.ui.motor_pos_ctrl_bar_2.valueChanged.connect(self.update_motor_pos_ctrl)

        # Set maximum value for the Pressure_ScrollBar
        self.ui.Pressure_ScrollBar.setMaximum(max_pressure/pressure_ctrl_resolution)
        self.ui.Pressure_ScrollBar.valueChanged.connect(self.update_pressure_ctrl)

    def switch_mode(self):
        textmode = self.ui.model_selection_comboBox.currentText()
        if textmode == "SoftMag":
            mode = "Sensing"
        elif textmode == "Gripper(Automatic)":
            mode = "Automatic"
        elif textmode == "Gripper(Testing)":
            mode = "Testing"
        if mode != self.current_mode:  # Only switch if the mode is different
            if self.mode_switch_worker is not None and self.mode_switch_worker.isRunning():
                return  # Prevent multiple switches at the same time

            self.mode_switch_worker = ModeSwitchWorker(mode)
            self.mode_switch_worker.mode_switched.connect(self.on_mode_switched)
            self.mode_switch_worker.start()
            self.current_mode = mode  # Update the current mode

    def on_mode_switched(self, mode, success):
        if success:
            rospy.loginfo(f"Successfully switched to {mode} mode")
        else:
            rospy.logerr(f"Failed to switch to {mode} mode")

    def publish_ctrl_mode(self, mode):
        ctrl_mode_message = json.dumps({"ctrl_mode": mode})
        self.ctrl_mode_publisher.publish(ctrl_mode_message)
        rospy.loginfo(f"Current mode: {mode}")

    def update_motor_pos_ctrl(self, position):
        # Temporarily block signals to prevent recursive updates
        self.ui.motor_pos_ctrl_bar_1.blockSignals(True)
        self.ui.motor_pos_ctrl_bar_2.blockSignals(True)

        # Set both sliders to the same position
        self.ui.motor_pos_ctrl_bar_1.setValue(position)
        self.ui.motor_pos_ctrl_bar_2.setValue(position)

        # Re-enable signals
        self.ui.motor_pos_ctrl_bar_1.blockSignals(False)
        self.ui.motor_pos_ctrl_bar_2.blockSignals(False)

        rospy.loginfo(f"Updating motor position to {position}")
        self.motor_pos_ctrl_publisher.publish(position)

    def zero_motor_callback(self):
        rospy.loginfo("Zero button pressed. Returning motor to zero position.")
        self.motor_pos_ctrl_publisher.publish(0)

    def update_pressure_ctrl(self, value):
        actual_input_pressure = value * pressure_ctrl_resolution
        self.ui.ctrl_pressure_val.display(actual_input_pressure)
        rospy.loginfo(f"Updating pressure to {actual_input_pressure} kPa")
        self.pressure_ctrl_publisher.publish(Float32(actual_input_pressure))

    @Slot(int)
    def onForceControlStateChanged(self, state):
        msg = String()
        doc = {
            "force_closure_enabled": state == Qt.Checked
        }
        msg.data = json.dumps(doc)
        self.force_closure_publisher.publish(msg)
        rospy.loginfo(f"Force control set to: {state == Qt.Checked}")

    @Slot(int)
    def onContactDetectStateChanged(self, state):
        msg = String()
        doc = {
            "contact_detect_enabled": state == Qt.Checked
        }
        msg.data = json.dumps(doc)
        rospy.loginfo(f"Contact detect set to: {state == Qt.Checked}")

    @Slot(int)
    def onSofaSimulationStateChanged(self, state):
        msg = String()
        doc = {
            "sofa_simulation_enabled": state == Qt.Checked
        }
        msg.data = json.dumps(doc)
        rospy.loginfo(f"SOFA simulation set to: {state == Qt.Checked}")

    def enable_nodes(self, nodes):
        for node in nodes:
            subprocess.Popen(['roslaunch', 'package_name', f'{node}.launch'])
            rospy.loginfo(f"Enabled node: {node}")

    def disable_nodes(self, nodes):
        for node in nodes:
            subprocess.Popen(['rosnode', 'kill', node])
            rospy.loginfo(f"Disabled node: {node}")

    def toggle_recording(self):
        if self.ui.record_data_button.text() == "Record data":
            self.ui.record_data_button.setText("Stop Recording")
            rospy.loginfo("Started recording data")
            self.record_data_publisher.publish(json.dumps({"record": True}))
        else:
            self.ui.record_data_button.setText("Record data")
            rospy.loginfo("Stopped recording data")
            self.record_data_publisher.publish(json.dumps({"record": False}))

    def update_sensor_data(self, data):
        # ROS callbacks are executed in a separate thread from the UI thread
        self.sensor_data_signal.emit(data.data) # use a signal to communicate with the UI thread safely

    # @Slot(str)
    def on_sensor_data_received(self, data):
        data = json.loads(data)
        sensor_id = data['sensor_id']
        filtered_data = data['filtered_data']
        stable_flag = data['stable_flag']

        # Update the corresponding sensor plot
        if len(filtered_data) == 3: # Check if filtered_data has exactly three elements
            x, y, z = filtered_data
            if sensor_id in self.sensor_plots:
                self.sensor_plots[sensor_id].update_plot(x, y, z)
        else:
            rospy.logerr("Filtered data does not contain all three components: {}".format(filtered_data))

        # Update the stable flag indicator
        if stable_flag:
            self.ui.stable_flag_indicator.setStyleSheet("background-color: green;")
        else:
            self.ui.stable_flag_indicator.setStyleSheet("")

    def update_prediction_results(self, msg):
        # Use a signal to safely update the UI from ROS callbacks
        self.prediction_results_signal.emit(msg.data)

    def on_prediction_results_received(self, data):
        result = json.loads(data)
        sensor_id = result.get('sensor_id')
        force = result['force']
        position = result['position']

        # Update the corresponding force plot based on the sensor ID
        self.force_plot.update_plot(sensor_id, force)

        # Update the LCD and plot based on sensor ID
        if sensor_id == 'S1':
            self.ui.force_val_1.display(force)
            self.quadrant_canvas_s1.update_quadrant(sensor_id, position, force)
        elif sensor_id == 'S2':
            self.ui.force_val_2.display(force)
            self.quadrant_canvas_s2.update_quadrant(sensor_id, position, force)

    def update_pressure_reading(self, data):
        pressure_value = float(data.data)  # Assuming the data is just a float, not JSON
        # Update your pressure plot here
        self.pressure_plot.update_plot(pressure_value)

    def release_callback(self):
        message = json.dumps({"release_flag": True})
        self.release_publisher.publish(message)

    def restart_callback(self):
        message = json.dumps({"restart_flag": True})
        self.restart_publisher.publish(message)

    def stop_all_callback(self):
        message = json.dumps({"stop_all_flag": True})
        self.stop_all_publisher.publish(message)

    def update_motor_pos_reading(self, data=None):
        motor_pos_reading = data.data
        # Assuming motor_pos_reading is the position value as an integer
        self.ui.motor_pos_reading_bar_1.setValue(motor_pos_reading)
        self.ui.motor_pos_reading_bar_2.setValue(motor_pos_reading)

    def grasp_stable_callback(self, data):
        data_json = json.loads(data.data)
        # Update UI for grasp stable indicator
        if data_json.get('grasp_stable_flag'):
            # You may need to use a Signal here to update the UI safely
            self.ui.grasp_stable_flag_indicator.setStyleSheet("background-color: green;")
        else:
            self.ui.grasp_stable_flag_indicator.setStyleSheet("")

    def motor_stop_callback(self, data):
        data_json = json.loads(data.data)
        # Update UI for motor stop indicator
        if data_json.get('motor_stop'):
            # You may need to use a Signal here to update the UI safely
            self.ui.motor_stop_indicator.setStyleSheet("background-color: orange;")
        else:
            self.ui.motor_stop_indicator.setStyleSheet("")

    def contact_detect_callback(self, data):
        data_json = json.loads(data.data)
        # Update UI for contact detect indicator
        if data_json.get('contact_detected'):
            # You may need to use a Signal here to update the UI safely
            self.ui.contact_detect_indicator.setStyleSheet("background-color: orange;")
        else:
            self.ui.contact_detect_indicator.setStyleSheet("")

    def signal_handler(self, sig, frame):
        print("SIGINT received. Shutting down gracefully...")
        QApplication.quit()
        rospy.signal_shutdown("SIGINT received")

if __name__ == "__main__":
    rospy.init_node('qt_gui')
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    signal.signal(signal.SIGINT, main_window.signal_handler)  # Register the signal handler
    sys.exit(app.exec())
