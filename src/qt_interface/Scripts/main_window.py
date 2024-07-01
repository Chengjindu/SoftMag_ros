#!/usr/bin/env python3
import sys
import signal
import json
import rospy
from std_srvs.srv import Trigger
import numpy as np
from std_msgs.msg import String, Int32, Float32, Float32MultiArray, Bool
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QSizePolicy, QCheckBox, QLCDNumber, QDoubleSpinBox
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
import matplotlib.colors as mcolors
from ui_mainwindow import Ui_MainWindow
import datetime
import os


np.random.BitGenerator = np.random.bit_generator

# Parameters to be set
sns.set_theme(context='notebook', style='whitegrid', palette='muted')   # Set the Seaborn style

steps_per_rev = 200     # Motor steps per revolute
full_stroke_angle = 85      # Gripper full stroll angle
max_steps = int((full_stroke_angle / 360) * steps_per_rev)

max_ctrl_pressure = 35       # Maximum scroll pressure for testing mode
pressure_ctrl_resolution = 0.1

sensor_plot_window_size = 1000      # sensor frequency ~50Hz
force_plot_window_size = 50          # force prediction frequency 15Hz
pressure_plot_window_size = 500     # pressure monitoring frequency ~30Hz

def get_config_param(param):
    # Adjust the path to locate start_config.json
    config_file = os.path.join(os.path.dirname(__file__), '../../../start_config.json')
    result = subprocess.run(['jq', '-r', f'.{param}', config_file], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.strip()
    else:
        raise Exception(f"Failed to get config param '{param}': {result.stderr}")


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


# Create a Matplotlib canvas class that can update the sensors plot dynamically
class SensorDataCanvas(FigureCanvas):
    def __init__(self, parent=None, window_size=1000, sensor_title=''):
        plt.style.use('ggplot')
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.fig.patch.set_facecolor((1, 1, 1, 0))  # Set figure background to transparent
        self.ax.set_facecolor((1, 1, 1, 0))  # Keep the axes background white
        super(SensorDataCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.window_size = window_size
        self.x_data, self.y_data, self.z_data = [], [], []
        self.max_data_points = window_size  # Limit the number of data points
        self.setup_plot(sensor_title)
        self.bg = None  # For blitting

    def setup_plot(self, sensor_title):
        self.fig.subplots_adjust(left = 0.15, right = 0.9, top = 0.9, bottom = 0.15)
        self.ax.set_title(sensor_title, fontsize = 10)
        self.ax.set_xlabel('Time', fontsize = 10)
        # self.ax.set_ylabel('Sensor Reading', fontsize=8)
        self.ax.grid(True, color='lightgrey', linewidth=2, alpha=0.4)  # Set grid color, line thickness, and transparency
        self.lines = {      # Initialize empty lines for each axis and store them
            'x': self.ax.plot(self.x_data, label='X-Axis', color='r')[0],
            'y': self.ax.plot(self.y_data, label='Y-Axis', color='g')[0],
            'z': self.ax.plot(self.z_data, label='Z-Axis', color='b')[0]
        }

        self.ax.legend(loc='upper right', handlelength = 2, handletextpad = 0.5, borderaxespad = 0.5, fontsize = 8)
        self.ax.annotate('(s)  ', xy=(1.05, -0.05), xycoords='axes fraction', ha='left', va='center')
        self.ax.annotate('(G)  ', xy=(-0.05, 1.02), xycoords='axes fraction', ha='center', va='bottom', rotation = 0)
        self.ax.set_xticklabels([])  # Hide x-axis tick labels
        self.draw()
        self.bg = self.copy_from_bbox(self.ax.bbox)  # Save background for blitting

    def update_plot(self, x, y, z):
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        # Limit the number of data points to avoid memory issues
        if len(self.x_data) > self.max_data_points:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.z_data.pop(0)

        self.lines['x'].set_data(range(len(self.x_data)), self.x_data)
        self.lines['y'].set_data(range(len(self.y_data)), self.y_data)
        self.lines['z'].set_data(range(len(self.z_data)), self.z_data)

        self.rescale_axes()
        self.redraw_plot()

    def rescale_axes(self):
        sample_number = len(self.x_data)    # Rescale x-axis to show a rolling window
        self.ax.set_xlim(max(0, sample_number - self.window_size), sample_number)

        # Dynamic y-axis scaling based on the visible data
        all_visible_data = self.x_data[-self.window_size:] + self.y_data[-self.window_size:] + self.z_data[-self.window_size:]
        y_min, y_max = min(all_visible_data), max(all_visible_data)
        self.ax.set_ylim(y_min - 0.1, y_max + 0.1)  # Add some padding

    def redraw_plot(self):
        if self.bg is None:  # Ensure bg is initialized before using it
            self.bg = self.copy_from_bbox(self.ax.bbox)
        self.restore_region(self.bg)  # Restore background
        for line in self.lines.values():
            self.ax.draw_artist(line)
        self.blit(self.ax.bbox)  # Blit updated region

def clear_plot(self):
        self.x_data, self.y_data, self.z_data = [], [], []
        self.lines['x'].set_data([], [])
        self.lines['y'].set_data([], [])
        self.lines['z'].set_data([], [])
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(-3, 3)
        self.draw()
        self.bg = self.copy_from_bbox(self.ax.bbox)  # Save background for blitting

# Create a Matplotlib canvas class that can update the quadrant plot dynamically
class QuadrantCanvas(FigureCanvas):
    def __init__(self, parent=None, base_color=(179/255, 229/255, 252/255, 0.2)):
        fig = Figure()  # Define the figure size
        self.fig = fig  # Assign fig to self.fig
        self.ax = fig.add_subplot(111, aspect='equal')
        self.fig.patch.set_facecolor((1, 1, 1, 0))  # Set figure background to transparent
        self.ax.set_facecolor((1, 1, 1, 0))  # Keep the axes background white
        self.ax.set_xlim(0, 1)  # Set the x-axis limits to be from 0 to 1
        self.ax.set_ylim(0, 1)  # Set the y-axis limits to be from 0 to 1
        self.ax.axis('off')  # Hide the axes

        # Define base color
        self.base_color = base_color

        self.quadrants = [  # Initialize the quadrant patches
            patches.Rectangle((0, 0), 0.5, 0.5, facecolor=self.base_color),
            patches.Rectangle((0, 0.5), 0.5, 0.5, facecolor=self.base_color),
            patches.Rectangle((0.5, 0.5), 0.5, 0.5, facecolor=self.base_color),
            patches.Rectangle((0.5, 0), 0.5, 0.5, facecolor=self.base_color)
        ]

        for quad in self.quadrants:  # Add quadrants to the axis
            self.ax.add_patch(quad)

        super(QuadrantCanvas, self).__init__(fig)
        self.setParent(parent)

    def create_gradient_color(self, base_color, force_value):
        # Create a color with transparency based on the force value.
        base_color = mcolors.to_rgba(base_color)  # Convert the base color to RGBA
        # Define a stronger color (darker version) for higher force values
        strong_color = (
        base_color[0] * 0.5, base_color[1] * 0.5, base_color[2] * 0.5, 1)  # Darker color with full opacity
        # Interpolate between base_color and strong_color based on force_value
        alpha = 0.2 + (force_value / 8.0) * 0.8  # Map force value (0-10) to transparency (0.2-1)
        alpha = max(0, min(alpha, 1))  # Ensure alpha is within the 0-1 range
        color = [base_color[i] + (strong_color[i] - base_color[i]) * (force_value / 10.0) for i in range(3)] + [alpha]
        return tuple(color)  # Return color with modified alpha

    def update_quadrant(self, sensor_id, prediction, force_value, touch_mode="single"):
        for quad in self.quadrants:  # Reset all quadrants to the default color
            quad.set_facecolor(self.base_color)

        # Update quadrants based on the force_value and prediction
        if force_value >= 0.5:  # Apply threshold to check for significant force
            if prediction is not None:
                color = self.create_gradient_color(self.base_color, force_value)  # Gradient for base color
                # Single touch mode
                if touch_mode == "single" and 1 <= prediction <= 4:
                    self.quadrants[prediction - 1].set_facecolor(color)

                # Multi-touch mode
                elif touch_mode == "multi":
                    for i, touch in enumerate(prediction):
                        if touch == 1:  # If touch is detected in the quadrant
                            self.quadrants[i].set_facecolor(color)

        self.draw()  # Redraw the canvas to show updated colors


# Create a Matplotlib canvas class that can update the force plot dynamically
class ForceDataCanvas(FigureCanvas):
    def __init__(self, parent = None, window_size = 50, title = ''):
        plt.style.use('ggplot')
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.fig.patch.set_facecolor((1, 1, 1, 0))  # Set figure background to transparent
        self.ax.set_facecolor((1, 1, 1, 0))  # Keep the axes background white
        super(ForceDataCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.window_size = window_size
        self.force_data_s1 = []
        self.force_data_s2 = []
        self.max_data_points = window_size  # Limit the number of data points
        self.setup_plot(title)
        self.bg = None  # For blitting

    def setup_plot(self, title):
        self.fig.subplots_adjust(left=0.15, right=0.9, top=0.9, bottom=0.15)
        self.ax.set_title(title, fontsize=10)
        self.ax.set_xlabel('Time', fontsize=10)
        self.ax.grid(True, color='lightgrey', linewidth=2, alpha=0.4)  # Set grid color, line thickness, and transparency

        self.force_line_s1, = self.ax.plot(self.force_data_s1, label='Force S1', color='m')
        self.force_line_s2, = self.ax.plot(self.force_data_s2, label='Force S2', color='c')

        self.ax.legend(loc='upper right', handlelength = 2, handletextpad = 0.5, borderaxespad = 0.5, fontsize = 8)
        self.ax.annotate(' (s)  ', xy=(1.05, -0.05), xycoords='axes fraction', ha='left', va='center')
        self.ax.annotate(' (N)  ', xy=(-0.05, 1.02), xycoords='axes fraction', ha='center', va='bottom', rotation=0)
        self.ax.set_xticklabels([])  # Hide x-axis tick labels
        self.draw()
        self.bg = self.copy_from_bbox(self.ax.bbox)  # Save background for blitting

    def update_plot(self, sensor_id, force):
        if sensor_id == 'S1':
            self.force_data_s1.append(force)
            if len(self.force_data_s1) > self.max_data_points:
                self.force_data_s1.pop(0)
            self.force_line_s1.set_data(range(len(self.force_data_s1)), self.force_data_s1)

        elif sensor_id == 'S2':
            self.force_data_s2.append(force)
            if len(self.force_data_s2) > self.max_data_points:
                self.force_data_s2.pop(0)
            self.force_line_s2.set_data(range(len(self.force_data_s2)), self.force_data_s2)

        self.rescale_axes()
        self.redraw_plot()

    def rescale_axes(self):
        sample_number = max(len(self.force_data_s1), len(self.force_data_s2))
        self.ax.set_xlim(max(0, sample_number - self.window_size), max(self.window_size, sample_number))

        all_force_data = self.force_data_s1[-self.window_size:] + self.force_data_s2[-self.window_size:]
        y_min, y_max = min(all_force_data), max(all_force_data)
        self.ax.set_ylim(y_min - 0.1, y_max + 0.1)

        if self.bg is None:
            self.bg = self.copy_from_bbox(self.ax.bbox)

    def redraw_plot(self):
        if self.bg is None:  # Ensure bg is initialized before using it
            self.bg = self.copy_from_bbox(self.ax.bbox)
        self.restore_region(self.bg)
        self.ax.draw_artist(self.force_line_s1)
        self.ax.draw_artist(self.force_line_s2)
        self.blit(self.ax.bbox)

    def clear_plot(self):
        self.force_data_s1 = []
        self.force_data_s2 = []
        self.force_line_s1.set_data([], [])
        self.force_line_s2.set_data([], [])
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(0, 6)
        self.draw()
        self.bg = self.copy_from_bbox(self.ax.bbox)  # Save background for blitting

# Create a Matplotlib canvas class that can update the pressure plot dynamically
class PressureDataCanvas(FigureCanvas):
    def __init__(self, parent = None, window_size = 2000, title = ''):
        plt.style.use('ggplot')  # Consistent style with the sensor plot
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.fig.patch.set_facecolor((1, 1, 1, 0))  # Set figure background to transparent
        self.ax.set_facecolor((1, 1, 1, 0))  # Keep the axes background white
        super(PressureDataCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.window_size = window_size
        self.pressure_data1 = []
        self.pressure_data2 = []
        self.max_data_points = window_size
        self.setup_plot(title)
        self.bg = None  # For blitting

    def setup_plot(self, title):
        self.fig.subplots_adjust(left=0.15, right=0.9, top=0.9, bottom=0.15)
        self.ax.set_title('Pressure Monitoring', fontsize=10)
        self.ax.set_xlabel('Time', fontsize=10)
        # self.ax.set_ylabel('Pressure (kPa)', fontsize=10)
        self.ax.grid(True, color='lightgrey', linewidth=2, alpha=0.4)  # Set grid color, line thickness, and transparency

        # Initialize an empty line for the pressure data
        self.pressure_line1, = self.ax.plot(self.pressure_data1, label='Pressure 1', color='purple')
        self.pressure_line2, = self.ax.plot(self.pressure_data2, label='Pressure 2', color='orange')
        self.ax.legend(loc='upper right', handlelength=2, handletextpad=0.5, borderaxespad=0.5, fontsize=8)
        self.ax.annotate(' (s)  ', xy=(1.05, -0.05), xycoords='axes fraction', ha='left', va='center')
        self.ax.annotate(' (kPa)  ', xy=(-0.05, 1.02), xycoords='axes fraction', ha='center', va='bottom', rotation=0)
        self.ax.set_xticklabels([])  # Hide x-axis tick labels

        self.draw()
        self.bg = self.copy_from_bbox(self.ax.bbox)  # Save background for blitting

    def update_plot(self, pressure_value1 = None, pressure_value2 = None):
        if pressure_value1 is not None:
            self.pressure_data1.append(pressure_value1)
            if len(self.pressure_data1) > self.max_data_points:
                self.pressure_data1.pop(0)
            self.pressure_line1.set_data(range(len(self.pressure_data1)), self.pressure_data1)

        if pressure_value2 is not None:
            self.pressure_data2.append(pressure_value2)
            if len(self.pressure_data2) > self.max_data_points:
                self.pressure_data2.pop(0)
            self.pressure_line2.set_data(range(len(self.pressure_data2)), self.pressure_data2)

        self.rescale_axes()
        self.redraw_plot()

    def rescale_axes(self):
        # Rescale x-axis to show a rolling window
        sample_number = len(self.pressure_data2)
        self.ax.set_xlim(max(0, sample_number - self.window_size), max(self.window_size, sample_number))

        # Dynamic y-axis scaling based on the visible data
        all_visible_data = self.pressure_data1[-self.window_size:] + self.pressure_data2[-self.window_size:]
        y_min, y_max = min(all_visible_data), max(all_visible_data)
        self.ax.set_ylim(y_min - 1, y_max + 1)  # Add some padding

    def redraw_plot(self):
        if self.bg is None:  # Ensure bg is initialized before using it
            self.bg = self.copy_from_bbox(self.ax.bbox)
        self.restore_region(self.bg)
        self.ax.draw_artist(self.pressure_line1)
        self.ax.draw_artist(self.pressure_line2)
        self.blit(self.ax.bbox)

    def clear_plot(self):
        self.pressure_data1 = []
        self.pressure_data2 = []
        self.pressure_line1.set_data([], [])
        self.pressure_line2.set_data([], [])
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(0, 40)  # Assuming pressure range from 0 to 100 kPa
        self.draw()
        self.bg = self.copy_from_bbox(self.ax.bbox)  # Save background for blitting

class ModeSwitchWorker(QThread):
    mode_switched = Signal(str, bool)

    def __init__(self, mode, parent=None):
        super(ModeSwitchWorker, self).__init__(parent)
        self.mode = mode

    def run(self):
        success = False
        if self.mode == "sensor":
            success = self.switch_to_sensor()
        elif self.mode == "gripper_automatic":
            success = self.switch_to_gripper_automatic()
        elif self.mode == "gripper_testing":
            success = self.switch_to_gripper_testing()
        self.mode_switched.emit(self.mode, success)

    def switch_to_sensor(self):
        rospy.wait_for_service('/switch_to_sensor')
        try:
            switch_service = rospy.ServiceProxy('/switch_to_sensor', Trigger)
            response = switch_service()
            if response.success:
                rospy.loginfo("Switched to sensing mode")
                return True
            else:
                rospy.logerr("Failed to switch to sensing mode: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return False

    def switch_to_gripper_automatic(self):
        rospy.wait_for_service('/switch_to_gripper_automatic')
        try:
            switch_service = rospy.ServiceProxy('/switch_to_gripper_automatic', Trigger)
            response = switch_service()
            if response.success:
                rospy.loginfo("Switched to gripper_automatic mode")
                return True
            else:
                rospy.logerr("Failed to switch to gripper_automatic mode: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return False

    def switch_to_gripper_testing(self):
        rospy.wait_for_service('/switch_to_gripper_testing')
        try:
            switch_service = rospy.ServiceProxy('/switch_to_gripper_testing', Trigger)
            response = switch_service()
            if response.success:
                rospy.loginfo("Switched to gripper_testing mode")
                return True
            else:
                rospy.logerr("Failed to switch to gripper_testing mode: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return False


class MainWindow(QMainWindow):
    sensor_data_signal = Signal(str)
    prediction_results_signal = Signal(str)
    pressure_reading1_signal = Signal(float)
    pressure_reading2_signal = Signal(float)
    elapsed_time = 0

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("SoftMag Interface")

        self.contact_states = [False] * 60  # List to store contact states (showing as the sensitivity of the contact detect visual interface)
        self.current_contact_status = None  # Initialize current contact status
        self.current_grasp_stable_status = None  # Initialize current grasp stable status
        self.current_motor_stop_status = None  # Initialize current motor stop status
        self.current_data_stable_flag = None  # Initialize current data stable flag

        self.current_force = {'S1': 0.0, 'S2': 0.0}
        self.current_position = {'S1': 0, 'S2': 0}

        # Initialize elapsed time timer
        self.elapsed_time_timer = QTimer(self)
        self.elapsed_time_timer.timeout.connect(self.update_elapsed_time)
        self.elapsed_time_timer.start(1000)

        self.update_interval = 2000  # Update interval for plotting (every 2 second)
        self.sensor_update_timer = QTimer()         # Create timers
        self.sensor_update_timer.timeout.connect(self.update_sensor_plots)
        self.sensor_update_timer.start(self.update_interval)

        self.prediction_update_timer = QTimer()
        self.prediction_update_timer.timeout.connect(self.update_prediction_plots)
        self.prediction_update_timer.start(self.update_interval)

        self.pressure_update_timer = QTimer()
        self.pressure_update_timer.timeout.connect(self.update_pressure_plots)
        self.pressure_update_timer.start(self.update_interval)

        self.mode_switch_worker = None
        self.current_mode = None  # Add this line to track the current mode

        # Initialize Global Setting Panel
        self.default_mode = get_config_param('default_pi_mode')
        self.set_default_comboBox(self.default_mode)
        self.ui.model_selection_comboBox.currentIndexChanged.connect(self.switch_mode)
        self.ui.max_pressure_input.valueChanged.connect(self.max_pressure_setting)

        signal.signal(signal.SIGINT, self.signal_handler)       # Signal handling for graceful shutdown

        # Create instances for the two sensors plot
        self.sensor_plots = {
            "S1": SensorDataCanvas(self.ui.S1_graph, window_size = sensor_plot_window_size, sensor_title='S1 Real-time Reading'),
            "S2": SensorDataCanvas(self.ui.S2_graph, window_size = sensor_plot_window_size, sensor_title='S2 Real-time Reading')
        }
        # Add the matplotlib canvas to the layout
        self.ui.S1_graph_layout = QVBoxLayout(self.ui.S1_graph)
        self.ui.S2_graph_layout = QVBoxLayout(self.ui.S2_graph)
        self.ui.S1_graph_layout.addWidget(self.sensor_plots["S1"])
        self.ui.S2_graph_layout.addWidget(self.sensor_plots["S2"])

        # Create instances for the two quadrant plots
        self.quadrant_canvas_s1 = QuadrantCanvas(self.ui.quadrant_1_1,
                                                 base_color=(179 / 255, 229 / 255, 252 / 255, 0.2))  # Light blue
        self.ui.quadrant_1_1_layout = QVBoxLayout(self.ui.quadrant_1_1)
        self.ui.quadrant_1_1_layout.addWidget(self.quadrant_canvas_s1)

        self.quadrant_canvas_s2 = QuadrantCanvas(self.ui.quadrant_2_2,
                                                 base_color=(255 / 255, 182 / 255, 193 / 255, 0.2))  # Light red
        self.ui.quadrant_2_2_layout = QVBoxLayout(self.ui.quadrant_2_2)
        self.ui.quadrant_2_2_layout.addWidget(self.quadrant_canvas_s2)

        # Create an instance of the ForceDataCanvas for plotting force values
        self.force_plot = ForceDataCanvas(self.ui.force_val_graph, window_size=force_plot_window_size, title='Real-time Force')
        self.ui.force_val_graph_layout = QVBoxLayout(self.ui.force_val_graph)
        self.ui.force_val_graph_layout.addWidget(self.force_plot)

        # Create instances for the two pressure plot
        self.pressure_plot = PressureDataCanvas(self.ui.pressure_graph, window_size=pressure_plot_window_size)
        self.ui.pressure_graph_layout = QVBoxLayout(self.ui.pressure_graph)
        self.ui.pressure_graph_layout.addWidget(self.pressure_plot)

        # Create instances for the motor position reading bar
        self.ui.motor_pos_reading_bar_1.setRange(0, max_steps)
        self.ui.motor_pos_reading_bar_2.setRange(0, max_steps)

        # Connect the existing checkboxes to the appropriate slots
        self.ui.forceControlCheckBox = self.findChild(QCheckBox, "force_feedback_ctrl_enable")
        self.ui.forceControlCheckBox.stateChanged.connect(self.onForceControlStateChanged)

        self.ui.sofaSimulationCheckBox = self.findChild(QCheckBox, "sofa_simulation_enable")
        self.ui.sofaSimulationCheckBox.stateChanged.connect(self.onSofaSimulationStateChanged)

        # Set up ROS subscribers
        rospy.Subscriber(f"/processed_sensor_data_S1", Float32MultiArray, self.update_sensor_data_S1)
        rospy.Subscriber(f"/processed_sensor_data_S2", Float32MultiArray, self.update_sensor_data_S2)
        rospy.Subscriber("/sensors_stabilized", Bool, self.update_data_stable_flag)
        rospy.Subscriber(f"/prediction_position_S1", Int32, self.update_prediction_position_S1)
        rospy.Subscriber(f"/prediction_position_S2", Int32, self.update_prediction_position_S2)
        rospy.Subscriber(f"/prediction_force_S1", Float32, self.update_prediction_force_S1)
        rospy.Subscriber(f"/prediction_force_S2", Float32, self.update_prediction_force_S2)
        rospy.Subscriber("/pressure_reading1", Float32, self.update_pressure_reading1)
        rospy.Subscriber("/pressure_reading2", Float32, self.update_pressure_reading2)
        rospy.Subscriber("/motor_zero", Bool, self.motor_zero_callback)
        rospy.Subscriber("/sensor_zero", Bool, self.sensor_zero_callback)
        rospy.Subscriber("/motor_pos_reading", Int32, self.update_motor_pos_reading)
        rospy.Subscriber("/contact_detect", Bool, self.contact_detect_callback)
        rospy.Subscriber("/grasp_stable", Bool, self.grasp_stable_callback)
        rospy.Subscriber("/motor_stop", Bool, self.motor_stop_callback)
        rospy.Subscriber('release_finish', Bool, self.release_finish_callback)

        self.is_contact_detected = False    # Initialize button states
        self.is_release_finished = False

        # Set up ROS publishers for the mode selection
        self.ctrl_mode_publisher = rospy.Publisher('/ctrl_mode', String, queue_size=10)

        # Set up ROS publishers for automatic grasping mode
        self.release_publisher = rospy.Publisher('/release', Bool, queue_size=10)
        self.restart_publisher = rospy.Publisher('/restart', Bool, queue_size=10)

        # Set up ROS publishers for testing mode
        self.max_pressure_publisher = rospy.Publisher('/max_pressure', Float32, queue_size=10)
        self.motor_pos_ctrl_publisher = rospy.Publisher('/motor_pos_ctrl', Int32, queue_size=10)
        self.pressure_ctrl_publisher = rospy.Publisher('/pressure_ctrl', Float32, queue_size=10)
        self.force_closure_publisher = rospy.Publisher('/force_closure', Bool, queue_size=10)
        self.zero_motor_publisher = rospy.Publisher('/zero_motor', Bool, queue_size=10)
        self.zero_sensor_publisher = rospy.Publisher('/zero_sensor', Bool, queue_size=10)
        self.record_data_publisher = rospy.Publisher('/record_data', Bool, queue_size=10)
        self.ctrl_coefficient_publisher = rospy.Publisher('/pressure_coefficient', Float32, queue_size=10)

        # Set up stop publishers for both modes
        self.stop_all_publisher = rospy.Publisher('/stop_all', Bool, queue_size=10)

        # Connect signals
        self.sensor_data_signal.connect(self.on_sensor_data_received)

        self.prediction_results_signal.connect(self.on_prediction_results_received)

        self.pressure_reading1_signal.connect(self.on_pressure_reading1_received)
        self.pressure_reading2_signal.connect(self.on_pressure_reading2_received)

        self.ui.release_button.clicked.connect(self.release_callback)
        self.ui.restart_button.clicked.connect(self.restart_callback)
        self.ui.stop_all_button.clicked.connect(self.stop_all_callback)
        self.ui.stop_all_button.setStyleSheet("""
                    QPushButton {
                        background-color: red;
                        color: white;
                        border: 2px solid darkred;
                        border-radius: 10px;
                        padding: 10px 20px;
                    }
                    QPushButton:hover {
                        background-color: crimson;
                    }
                    QPushButton:pressed {
                        background-color: darkred;
                    }
                """)
        self.ui.stop_all_button.style().unpolish(self.ui.stop_all_button)
        self.ui.stop_all_button.style().polish(self.ui.stop_all_button)

        self.ui.motor_pos_ctrl_bar_1.valueChanged.connect(self.update_motor_pos_ctrl)
        self.ui.motor_pos_ctrl_bar_2.valueChanged.connect(self.update_motor_pos_ctrl)
        self.ui.motor_zero_button.clicked.connect(self.zero_motor_callback)
        self.ui.sensor_zero_button.clicked.connect(self.zero_sensor_callback)
        self.ui.ctrl_coefficient.valueChanged.connect(self.publish_ctrl_coefficient)
        self.ui.record_data_button.clicked.connect(self.toggle_recording)
        self.ui.record_data_button.setStyleSheet("""
                    QPushButton {
                        background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                                    stop:0 rgba(179, 229, 252, 0.7), 
                                                    stop:1 rgba(255, 192, 192, 0.7));
                        border: none;
                        border-radius: 5px;
                        padding: 5px;
                    }
                    QPushButton:hover {
                        background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                                    stop:0 rgba(179, 229, 252, 0.9), 
                                                    stop:1 rgba(255, 192, 192, 0.9));
                    }
                    QPushButton:pressed {
                        background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                                    stop:0 rgba(179, 229, 252, 0.5), 
                                                    stop:1 rgba(255, 192, 192, 0.5));
                    }
                """)
        self.ui.record_data_button.style().unpolish(self.ui.record_data_button)
        self.ui.record_data_button.style().polish(self.ui.record_data_button)

        # Set maximum value for the pressure_scrollbar
        self.ui.pressure_scrollbar.setMaximum(max_ctrl_pressure/pressure_ctrl_resolution)
        self.ui.pressure_scrollbar.valueChanged.connect(self.scrollbar_to_spinbox)
        self.ui.pressure_type_input.valueChanged.connect(self.spinbox_to_scrollbar)

        # Connect the time reset button
        self.ui.time_reset_button.clicked.connect(self.reset_elapsed_time)

    def set_default_comboBox(self, mode):
        if mode == 'sensor':
            self.ui.model_selection_comboBox.addItem("SoftMag")
            self.ui.model_selection_comboBox.addItem("Gripper(Automatic)")
            self.ui.model_selection_comboBox.addItem("Gripper(Testing)")
        elif mode == 'gripper_automatic':
            self.ui.model_selection_comboBox.addItem("Gripper(Automatic)")
            self.ui.model_selection_comboBox.addItem("Gripper(Testing)")
            self.ui.model_selection_comboBox.addItem("SoftMag")
        elif mode == 'gripper_testing':
            self.ui.model_selection_comboBox.addItem("Gripper(Testing)")
            self.ui.model_selection_comboBox.addItem("Gripper(Automatic)")
            self.ui.model_selection_comboBox.addItem("SoftMag")

    def switch_mode(self):
        textmode = self.ui.model_selection_comboBox.currentText()
        if textmode == "SoftMag":
            mode = "sensor"
        elif textmode == "Gripper(Automatic)":
            mode = "gripper_automatic"
        elif textmode == "Gripper(Testing)":
            mode = "gripper_testing"
        if mode != self.current_mode:  # Only switch if the mode is different
            if self.mode_switch_worker is not None and self.mode_switch_worker.isRunning():
                return  # Prevent multiple switches at the same time

            self.mode_switch_worker = ModeSwitchWorker(mode)
            self.mode_switch_worker.mode_switched.connect(self.on_mode_switched)
            self.mode_switch_worker.start()
            self.current_mode = mode  # Update the current mode
            rospy.loginfo(f"Started mode switch worker for mode: {mode}")
        else:
            rospy.loginfo("Requested mode is already active. No action taken.")

    def max_pressure_setting(self, value):
        self.max_pressure_publisher.publish(Float32(value))
        max_value = int(value / pressure_ctrl_resolution)
        self.ui.pressure_scrollbar.setMaximum(max_value)
        self.ui.pressure_type_input.setMaximum(value)

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
        msg = Bool()
        if self.ui.motor_zero_button.text() == "Zero Motor":
            self.ui.motor_zero_button.setText("Zeroing...")
            msg.data = True
        else:
            msg.data = False
        self.zero_motor_publisher.publish(msg)

    def motor_zero_callback(self, data):
        if data.data:
            self.ui.motor_zero_button.setText("Motor At Zero")
            self.ui.motor_zero_button.setStyleSheet("background-color: lightblue")
        else:
            self.ui.motor_zero_button.setText("Zero Motor")
            self.ui.motor_zero_button.setStyleSheet("background-color: lightgray")

    def zero_sensor_callback(self):
        msg = Bool()
        if self.ui.sensor_zero_button.text() == "Zero Sensor":
            self.ui.sensor_zero_button.setText("Zeroing...")
            msg.data = True
        else:
            msg.data = False
        self.zero_sensor_publisher.publish(msg)

    def sensor_zero_callback(self, data):
        if data.data:
            self.ui.sensor_zero_button.setText("Zero Sensor")
            self.ui.sensor_zero_button.setStyleSheet("background-color: lightgray")

    def scrollbar_to_spinbox(self, value):
        max_pressure = self.ui.max_pressure_input.value()
        actual_input_pressure = min(value * pressure_ctrl_resolution, max_pressure)
        self.ui.pressure_type_input.blockSignals(True)
        self.ui.pressure_type_input.setValue(actual_input_pressure)
        self.ui.pressure_type_input.blockSignals(False)
        self.pressure_ctrl_publisher.publish(Float32(actual_input_pressure))

    def spinbox_to_scrollbar(self, value):
        max_pressure = self.ui.max_pressure_input.value()
        actual_input_pressure = min(value, max_pressure)
        scrollbar_value = int(actual_input_pressure / pressure_ctrl_resolution)
        self.ui.pressure_scrollbar.blockSignals(True)
        self.ui.pressure_scrollbar.setValue(scrollbar_value)
        self.ui.pressure_scrollbar.blockSignals(False)
        self.pressure_ctrl_publisher.publish(Float32(actual_input_pressure))

    def publish_ctrl_coefficient(self, value):
        rospy.loginfo(f"Publishing control coefficient: {value}")
        self.ctrl_coefficient_publisher.publish(Float32(value))

    @Slot(int)
    def onForceControlStateChanged(self, state):
        msg = Bool()
        msg.data = (state == Qt.Checked)
        self.force_closure_publisher.publish(msg)
        rospy.loginfo(f"Force control set to: {state == Qt.Checked}")

    @Slot(int)
    def onSofaSimulationStateChanged(self, state):
        msg = Bool()
        msg.data = (state == Qt.Checked)
        self.sofaSimulationCheckBox.publish(msg)
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
        rospy.wait_for_service('/record_data')
        try:
            record_service = rospy.ServiceProxy('/record_data', Trigger)
            response = record_service()
            if response.success:
                if self.ui.record_data_button.text() == "Record Data":
                    self.ui.record_data_button.setText("Stop Recording")
                    self.ui.record_data_button.setStyleSheet("""
                            QPushButton {
                                background-color: red;
                                color: white;
                                border: none;
                                border-radius: 5px;
                                padding: 5px;
                            }
                            QPushButton:hover {
                                background-color: darkred;
                            }
                            QPushButton:pressed {
                                background-color: crimson;
                            }
                        """)
                else:
                    self.ui.record_data_button.setText("Record Data")
                    self.ui.record_data_button.setStyleSheet("""
                            QPushButton {
                                background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                                            stop:0 rgba(179, 229, 252, 0.7), 
                                                            stop:1 rgba(255, 192, 192, 0.7));
                                border: none;
                                border-radius: 5px;
                                padding: 5px;
                            }
                            QPushButton:hover {
                                background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                                            stop:0 rgba(179, 229, 252, 0.9), 
                                                            stop:1 rgba(255, 192, 192, 0.9));
                            }
                            QPushButton:pressed {
                                background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                                            stop:0 rgba(179, 229, 252, 0.5), 
                                                            stop:1 rgba(255, 192, 192, 0.5));
                            }
                        """)
                rospy.loginfo(response.message)
            else:
                rospy.logerr("Failed to toggle recording: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def restart_callback(self):
        if self.ui.restart_button.text() in ["Start", "Restart"]:
            msg = Bool()
            msg.data = True
            self.restart_publisher.publish(msg)
            self.ui.restart_button.setText("Closing")
            self.ui.restart_button.setStyleSheet("background-color: lime;")

    def release_callback(self):
        msg = Bool()
        msg.data = True
        self.release_publisher.publish(msg)
        if self.ui.restart_button.text() == "Pressurizing":
            self.ui.restart_button.setText("Releasing")
            self.ui.restart_button.setStyleSheet("background-color: blue;")

    def release_finish_callback(self, data):
        self.is_release_finished = data.data
        if self.is_release_finished and self.ui.restart_button.text() == "Releasing":
            self.ui.restart_button.setText("Restart")
            self.ui.restart_button.setStyleSheet("background-color: lightgray;")

    def stop_all_callback(self):
        msg = Bool()
        msg.data = True
        self.stop_all_publisher.publish(msg)
        self.ui.stop_all_button.setStyleSheet("""
                QPushButton {
                    background-color: red;
                    color: white;
                    border: 2px solid darkred;
                    border-radius: 10px;
                    padding: 10px 20px;
                }
                QPushButton:hover {
                    background-color: crimson;
                }
                QPushButton:pressed {
                    background-color: darkred;
                }
            """)
        self.ui.restart_button.setText("Restart")
        self.ui.restart_button.setStyleSheet("background-color: lightgray;")

    def update_motor_pos_reading(self, data=None):
        motor_pos_reading = data.data
        # Assuming motor_pos_reading is the position value as an integer
        self.ui.motor_pos_reading_bar_1.setValue(motor_pos_reading)
        self.ui.motor_pos_reading_bar_2.setValue(motor_pos_reading)

    def contact_detect_callback(self, data):
        new_contact_status = data.data
        # Update the list of contact states
        self.contact_states.pop(0)  # Remove the oldest state
        self.contact_states.append(new_contact_status)  # Add the new state

        # Determine the overall contact status
        overall_contact_status = any(self.contact_states)  # True if any state in the list is True

        if overall_contact_status != self.current_contact_status:  # Only update if there is a change
            self.current_contact_status = overall_contact_status
            if self.current_contact_status and self.ui.restart_button.text() == "Closing":
                self.ui.restart_button.setText("Pressurizing")
                self.ui.restart_button.setStyleSheet("background-color: salmon;")

            if not self.current_contact_status:
                self.ui.contact_detect_indicator.setProperty("class", "")
                self.ui.contact_detect_indicator.setText("No Contact")
                self.ui.contact_detect_indicator.setStyleSheet("""
                    QLabel#contact_detect_indicator {
                        background-color: lightgray; /* Light gray background */
                        color: black; /* Black text */
                        border: 1px solid gray; /* Gray border */
                        border-radius: 5px; /* Rounded corners */
                        padding: 5px; /* Padding */
                        min-width: 100px; /* Minimum width */
                        text-align: center; /* Center text alignment */
                    }
                """)
            else:
                self.ui.contact_detect_indicator.setText("Contact Detected")
                self.ui.contact_detect_indicator.setProperty("class", "active")
                self.ui.contact_detect_indicator.setStyleSheet("""
                    QLabel#contact_detect_indicator {
                        background-color: lightgray; /* Light gray background */
                        color: black; /* Black text */
                        border: 1px solid gray; /* Gray border */
                        border-radius: 5px; /* Rounded corners */
                        padding: 5px; /* Padding */
                        min-width: 100px; /* Minimum width */
                        text-align: center; /* Center text alignment */
                    }
                    QLabel#contact_detect_indicator.active {
                        background-color: orange;
                        border: 1px solid #B8860B;
                        color: white;
                    }
                """)
            self.ui.contact_detect_indicator.style().unpolish(self.ui.contact_detect_indicator)
            self.ui.contact_detect_indicator.style().polish(self.ui.contact_detect_indicator)

    def grasp_stable_callback(self, data):
        new_grasp_stable_status = data.data
        if new_grasp_stable_status != self.current_grasp_stable_status:  # Only update if there is a change
            self.current_grasp_stable_status = new_grasp_stable_status
            if self.current_grasp_stable_status:
                self.ui.grasp_stable_flag_indicator.setProperty("class", "active")
                self.ui.grasp_stable_flag_indicator.setText("Data Stable")
                self.ui.grasp_stable_flag_indicator.setStyleSheet("""
                    QLabel#grasp_stable_flag_indicator {
                        background-color: lightgray; /* Light gray background */
                        color: black; /* Black text */
                        border: 1px solid gray; /* Gray border */
                        border-radius: 5px; /* Rounded corners */
                        padding: 5px; /* Padding */
                        min-width: 100px; /* Minimum width */
                        text-align: center; /* Center text alignment */
                    }
                    QLabel#grasp_stable_flag_indicator.active {
                        background-color: #2ecc71;
                        border: 1px solid #27ae60;
                        color: white;
                    }
                """)
            else:
                self.ui.grasp_stable_flag_indicator.setProperty("class", "")
                self.ui.grasp_stable_flag_indicator.setText("Data Processing")
                self.ui.grasp_stable_flag_indicator.setStyleSheet("""
                    QLabel#grasp_stable_flag_indicator {
                        background-color: lightgray; /* Light gray background */
                        color: black; /* Black text */
                        border: 1px solid gray; /* Gray border */
                        border-radius: 5px; /* Rounded corners */
                        padding: 5px; /* Padding */
                        min-width: 100px; /* Minimum width */
                        text-align: center; /* Center text alignment */
                    }
                """)

    def motor_stop_callback(self, data):
        new_motor_stop_status = data.data
        if new_motor_stop_status != self.current_motor_stop_status:  # Only update if there is a change
            self.current_motor_stop_status = new_motor_stop_status
            if self.current_motor_stop_status:
                self.ui.motor_stop_indicator.setProperty("class", "")
                self.ui.motor_stop_indicator.setText("Motor Stopped")
                self.ui.motor_stop_indicator.setStyleSheet("""
                    QLabel#motor_stop_indicator {
                        background-color: lightcyan;
                        color: black;
                        border: 1px solid #00ced1; /* Dark Cyan border */
                        border-radius: 5px;
                        padding: 5px;
                        min-width: 100px;
                        text-align: center;
                    }
                """)
            else:
                self.ui.motor_stop_indicator.setProperty("class", "active")
                self.ui.motor_stop_indicator.setText("Motor running")
                self.ui.motor_stop_indicator.setStyleSheet("""
                    QLabel#motor_stop_indicator {
                        background-color: lightgray; /* Light gray background */
                        color: black; /* Black text */
                        border: 1px solid gray; /* Gray border */
                        border-radius: 5px; /* Rounded corners */
                        padding: 5px; /* Padding */
                        min-width: 100px; /* Minimum width */
                        text-align: center; /* Center text alignment */
                    }
                    QLabel#motor_stop_indicator.active {
                        background-color: orange;
                        border: 1px solid #B8860B;
                        color: white;
                    }
                """)

    def update_elapsed_time(self):
        self.elapsed_time += 1
        formatted_time = str(datetime.timedelta(seconds = self.elapsed_time))
        self.ui.elapsed_time_display.display(formatted_time)

    def reset_elapsed_time(self):
        self.elapsed_time = 0
        formatted_time = str(datetime.timedelta(seconds = self.elapsed_time))
        self.ui.elapsed_time_display.display(formatted_time)

    def update_sensor_data_S1(self, data):
        self.sensor_data_signal.emit(json.dumps({"sensor_id": "S1", "data": data.data}))

    def update_sensor_data_S2(self, data):
        self.sensor_data_signal.emit(json.dumps({"sensor_id": "S2", "data": data.data}))

    # @Slot(str)
    def on_sensor_data_received(self, data):
        data = json.loads(data)
        sensor_id = data['sensor_id']
        sensor_data = data['data']

        # Update the corresponding sensor plot
        if len(sensor_data) == 3: # Check if filtered_data has exactly three elements
            x, y, z = sensor_data
            if sensor_id in self.sensor_plots:
                self.sensor_plots[sensor_id].update_plot(x, y, z)
        else:
            rospy.logerr("Filtered data does not contain all three components: {}".format(sensor_data))

    def update_data_stable_flag(self, msg):
        new_data_stable_flag = msg.data
        if new_data_stable_flag != self.current_data_stable_flag:  # Only update if there is a change
            self.current_data_stable_flag = new_data_stable_flag
            if self.current_data_stable_flag:
                self.ui.data_stable_flag_indicator.setProperty("class", "active")
                self.ui.data_stable_flag_indicator.setText("Data Stable")
                self.ui.data_stable_flag_indicator.setStyleSheet("""
                            QLabel#data_stable_flag_indicator {
                                background-color: lightgray; /* Light gray background */
                                color: black; /* Black text */
                                border: 1px solid gray; /* Gray border */
                                border-radius: 5px; /* Rounded corners */
                                padding: 5px; /* Padding */
                                min-width: 100px; /* Minimum width */
                                text-align: center; /* Center text alignment */
                            }
                            QLabel#data_stable_flag_indicator.active {
                                background-color: #2ecc71;
                                border: 1px solid #27ae60;
                                color: white;
                            }
                        """)
            else:
                self.ui.data_stable_flag_indicator.setProperty("class", "")
                self.ui.data_stable_flag_indicator.setText("Data Processing")
                self.ui.data_stable_flag_indicator.setStyleSheet("""
            QLabel#data_stable_flag_indicator {
                background-color: lightgray; /* Light gray background */
                color: black; /* Black text */
                border: 1px solid gray; /* Gray border */
                border-radius: 5px; /* Rounded corners */
                padding: 5px; /* Padding */
                min-width: 100px; /* Minimum width */
                text-align: center; /* Center text alignment */
            }
        """)

    def update_prediction_position_S1(self, msg):
        self.prediction_results_signal.emit(json.dumps({"sensor_id": "S1", "type": "position", "data": msg.data}))

    def update_prediction_position_S2(self, msg):
        self.prediction_results_signal.emit(json.dumps({"sensor_id": "S2", "type": "position", "data": msg.data}))

    def update_prediction_force_S1(self, msg):
        self.prediction_results_signal.emit(json.dumps({"sensor_id": "S1", "type": "force", "data": msg.data}))

    def update_prediction_force_S2(self, msg):
        self.prediction_results_signal.emit(json.dumps({"sensor_id": "S2", "type": "force", "data": msg.data}))

    def get_current_force(self, sensor_id):
        return self.current_force[sensor_id]

    def get_current_position(self, sensor_id):
        return self.current_position[sensor_id]

    def set_current_force(self, sensor_id, value):
        self.current_force[sensor_id] = value

    def set_current_position(self, sensor_id, value):
        self.current_position[sensor_id] = value

    def on_prediction_results_received(self, data):
        result = json.loads(data)
        sensor_id = result.get('sensor_id')
        data_type = result.get('type')
        value = result.get('data')

        if data_type == 'position':
            position = int(value)
            self.set_current_position(sensor_id, position)
            force = self.get_current_force(sensor_id)
        elif data_type == 'force':
            force = float(value)
            self.set_current_force(sensor_id, force)
            position = self.get_current_position(sensor_id)

        # Update the corresponding force plot based on the sensor ID
        self.force_plot.update_plot(sensor_id, force)

        # Update the LCD and plot based on sensor ID
        if sensor_id == 'S1':
            self.ui.force_val_1.display(force)
            self.quadrant_canvas_s1.update_quadrant(sensor_id, position, force)
        elif sensor_id == 'S2':
            self.ui.force_val_2.display(force)
            self.quadrant_canvas_s2.update_quadrant(sensor_id, position, force)

    def update_pressure_reading1(self, data):
        pressure_value = float(data.data)
        self.pressure_reading1_signal.emit(pressure_value)

    def update_pressure_reading2(self, data):
        pressure_value = float(data.data)
        self.pressure_reading2_signal.emit(pressure_value)

    # @Slot(float)
    def on_pressure_reading1_received(self, pressure_value):
        self.pressure_plot.update_plot(pressure_value1 = pressure_value)

    # @Slot(float)
    def on_pressure_reading2_received(self, pressure_value):
        self.pressure_plot.update_plot(pressure_value2 = pressure_value)

    def update_sensor_plots(self):
        for sensor_id, sensor_plot in self.sensor_plots.items():
            sensor_plot.draw()

    def update_prediction_plots(self):
        self.force_plot.draw()

    def update_pressure_plots(self):
        self.pressure_plot.draw()

    def signal_handler(self, sig, frame):
        print("SIGINT received. Shutting down gracefully...")
        QApplication.quit()
        rospy.signal_shutdown("SIGINT received")

if __name__ == "__main__":
    rospy.init_node('main_interface')
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    signal.signal(signal.SIGINT, main_window.signal_handler)  # Register the signal handler
    sys.exit(app.exec())
