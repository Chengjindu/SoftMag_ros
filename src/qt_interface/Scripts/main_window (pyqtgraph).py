# This Python file uses the following encoding: utf-8
# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
import sys
import json
import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float32
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout
from PySide6.QtCore import Signal, Slot, QTimer
from PySide6.QtGui import QColor
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.figure import Figure
import pyqtgraph as pg
from pyqtgraph import ArrowItem
from ui_mainwindow import Ui_MainWindow

steps_per_rev = 200
full_stroke_angle = 85
max_steps = int((full_stroke_angle / 360) * steps_per_rev)

class SensorDataPlot(pg.PlotWidget):
    def __init__(self, parent=None, window_size=30):
        super(SensorDataPlot, self).__init__(parent)
        self.window_size = window_size  # seconds
        self.plotItem = self.getPlotItem()
        self.plotItem.showGrid(x=True, y=True)
        self.data_x = np.array([])
        self.data_y = np.array([])
        self.data_z = np.array([])

        # Initialize plot range
        self.plotItem.setXRange(0, window_size)
        self.plotItem.setYRange(-3, 2)
        self.plot_lines = {
            'x': self.plotItem.plot(pen=pg.mkPen('r', width=3)),
            'y': self.plotItem.plot(pen=pg.mkPen('g', width=3)),
            'z': self.plotItem.plot(pen=pg.mkPen('b', width=3))
        }
        self.legend = pg.LegendItem((100, 60), offset=(-1, 1))  # args are (size, position)
        self.legend.setParentItem(self.plotItem)

        self.legend.addItem(self.plot_lines['x'], 'X-Axis')
        self.legend.addItem(self.plot_lines['y'], 'Y-Axis')
        self.legend.addItem(self.plot_lines['z'], 'Z-Axis')

        self.setBackground('w')
        # self.plotItem.getAxis('left').setPen('k')
        # self.plotItem.getAxis('bottom').setPen('k')

        arrow_right = ArrowItem(angle=180, headLen=15, pen={'color': 'k', 'width': 2})
        arrow_top = ArrowItem(angle=90, headLen=15, pen={'color': 'k', 'width': 2})
        arrow_right.setPos(window_size, 0)  # Place it at the right end of the x-axis
        arrow_top.setPos(0, 100)  # Place it at the top end of the y-axis
        self.plotItem.addItem(arrow_right)
        self.plotItem.addItem(arrow_top)

    def update_arrow_positions(self):
        # Get the current view range
        view_range = self.viewRect()

        # Create or update arrow items for the end of each axis
        if not hasattr(self, 'arrow_right'):
            self.arrow_right = ArrowItem(angle=0, headLen=15, pen={'color': 'k', 'width': 2})
            self.plotItem.addItem(self.arrow_right)
        if not hasattr(self, 'arrow_top'):
            self.arrow_top = ArrowItem(angle=-90, headLen=15, pen={'color': 'k', 'width': 2})
            self.plotItem.addItem(self.arrow_top)

        # Set position for the right arrow based on the current x-axis maximum
        self.arrow_right.setPos(view_range.right(), view_range.bottom())

        # Set position for the top arrow based on the current y-axis maximum
        self.arrow_top.setPos(view_range.left(), view_range.top())

    def autoscale_y_axis(self):
        x_min = max(0, len(self.data_x) - self.window_size)
        data_visible = np.concatenate((self.data_x[x_min:], self.data_y[x_min:], self.data_z[x_min:]))  # Add x_data if needed

        y_min = np.min(data_visible) - 0.1  # Add a small buffer
        y_max = np.max(data_visible) + 0.1
        self.plotItem.setYRange(y_min, y_max)

    def update_plot(self, sensor_id, x, y, z):
        self.data_x = np.append(self.data_x[-self.window_size:], x)
        self.data_y = np.append(self.data_y[-self.window_size:], y)
        self.data_z = np.append(self.data_z[-self.window_size:], z)

        # Update the plot data
        self.plot_lines['x'].setData(self.data_x)
        self.plot_lines['y'].setData(self.data_y)
        self.plot_lines['z'].setData(self.data_z)

        # Adjust the x-axis to implement scrolling effect
        max_x = len(self.data_x)  # Assuming data points are spaced at 1 second intervals
        min_x = max(0, max_x - self.window_size)
        self.plotItem.setXRange(min_x, max_x)

        self.autoscale_y_axis() # Implement dynamic y-axis rescaling based on the visible data range
        self.update_arrow_positions()   # Update arrow positions

    def clear_plot(self):
        self.data_x = np.array([])
        self.data_y = np.array([])
        self.data_z = np.array([])
        # Clear more data arrays for z and other sensor data if needed
        for line in self.plot_lines.values():
            line.clear()

class PressureDataPlot(pg.PlotWidget):
    def __init__(self, parent=None, window_size=30):
        super(PressureDataPlot, self).__init__(parent)
        self.window_size = window_size  # The same window size for scrolling effect
        self.plotItem = self.getPlotItem()
        self.plotItem.showGrid(x=True, y=True)
        self.data_pressure = np.array([])

        # Initialize plot range with a dynamic Y range
        self.plotItem.setXRange(0, window_size)
        self.plotItem.setYRange(0, 100)  # Adjust based on expected pressure range
        self.pressure_line = self.plotItem.plot(pen=pg.mkPen('b', width=3))

        self.setBackground('w')
        self.plotItem.getAxis('left').setPen('k')
        self.plotItem.getAxis('bottom').setPen('k')

        arrow_right = ArrowItem(angle=0, headLen=15, pen={'color': 'k', 'width': 2})
        arrow_top = ArrowItem(angle=-90, headLen=15, pen={'color': 'k', 'width': 2})
        arrow_right.setPos(window_size, 0)  # Place it at the right end of the x-axis
        arrow_top.setPos(0, 100)  # Place it at the top end of the y-axis
        self.plotItem.addItem(arrow_right)
        self.plotItem.addItem(arrow_top)

    def update_arrow_positions(self):
        # Get the current view range
        view_range = self.viewRect()

        # Create or update arrow items for the end of each axis
        if not hasattr(self, 'arrow_right'):
            self.arrow_right = ArrowItem(angle=0, headLen=15, pen={'color': 'k', 'width': 2})
            self.plotItem.addItem(self.arrow_right)
        if not hasattr(self, 'arrow_top'):
            self.arrow_top = ArrowItem(angle=-90, headLen=15, pen={'color': 'k', 'width': 2})
            self.plotItem.addItem(self.arrow_top)

        # Set position for the right arrow based on the current x-axis maximum
        self.arrow_right.setPos(view_range.right(), view_range.bottom())

        # Set position for the top arrow based on the current y-axis maximum
        self.arrow_top.setPos(view_range.left(), view_range.top())

    def autoscale_y_axis(self):
        # Autoscale Y based on visible data
        x_min = max(0, len(self.data_pressure) - self.window_size)
        data_visible = self.data_pressure[x_min:]

        y_min = np.min(data_visible) - 0.1  # Add a small buffer
        y_max = np.max(data_visible) + 0.1
        self.plotItem.setYRange(y_min, y_max)

    def update_plot(self, pressure_value):
        self.data_pressure = np.append(self.data_pressure[-self.window_size:], pressure_value)

        # Update the plot data
        self.pressure_line.setData(np.arange(len(self.data_pressure)), self.data_pressure)

        # Adjust the x-axis to implement scrolling effect
        max_x = len(self.data_pressure)
        min_x = max(0, max_x - self.window_size)
        self.plotItem.setXRange(min_x, max_x)

        self.update_arrow_positions()
        self.autoscale_y_axis() # Autoscale the y-axis

    def clear_plot(self):
        self.data_pressure = np.array([])
        self.pressure_line.clear()


class MainWindow(QMainWindow):
    sensor_data_signal = Signal(str)


    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.sensor_plots = {   # Replace QFrame with matplotlib canvas
            "S1": SensorDataPlot(self.ui.S1_graph),
            "S2": SensorDataPlot(self.ui.S2_graph)
        }

        # Add the matplotlib canvas to the layout
        self.ui.S1_graph_layout = QVBoxLayout(self.ui.S1_graph)
        self.ui.S2_graph_layout = QVBoxLayout(self.ui.S2_graph)
        self.ui.S1_graph_layout.addWidget(self.sensor_plots["S1"])
        self.ui.S2_graph_layout.addWidget(self.sensor_plots["S2"])
        self.ui.motor_pos_bar_1.setRange(0, max_steps)
        self.ui.motor_pos_bar_2.setRange(0, max_steps)

        self.pressure_plot = PressureDataPlot(self.ui.pressure_graph, window_size=30)

        self.ui.pressure_graph_layout = QVBoxLayout(self.ui.pressure_graph)
        self.ui.pressure_graph_layout.addWidget(self.pressure_plot)

        # Set up ROS subscribers
        rospy.Subscriber("/processed_sensor_data", String, self.update_sensor_data)
        rospy.Subscriber("/pressure", Float32, self.update_pressure)
        rospy.Subscriber("/motor_pos", Int32, self.update_motor_pos)

        rospy.Subscriber("/contact_detect", String, self.contact_detect_callback)
        rospy.Subscriber("/grasp_stable", String, self.grasp_stable_callback)
        rospy.Subscriber("/motor_stop", String, self.motor_stop_callback)

        # Connect signals
        self.sensor_data_signal.connect(self.on_sensor_data_received)

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
                self.sensor_plots[sensor_id].update_plot(sensor_id, x, y, z)
        else:
            rospy.logerr("Filtered data does not contain all three components: {}".format(filtered_data))

        # Update the stable flag indicator
        if stable_flag:
            self.ui.stable_flag_indicator.setStyleSheet("background-color: green;")
        else:
            self.ui.stable_flag_indicator.setStyleSheet("")

    def update_sensor_data(self, data):
        # ROS callbacks are executed in a separate thread from the UI thread
        self.sensor_data_signal.emit(data.data) # We use a signal to communicate with the UI thread safely

    def update_pressure(self, data):
        pressure_value = float(data.data)  # Assuming the data is just a float, not JSON
        # Update your pressure plot here
        self.pressure_plot.update_plot(pressure_value)

    def update_motor_pos(self, data=None):
        motor_pos = data.data
        # Assuming motor_pos is the position value as an integer
        self.ui.motor_pos_bar_1.setValue(motor_pos)
        self.ui.motor_pos_bar_2.setValue(motor_pos)

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
        if data_json.get('change_flag'):
            # You may need to use a Signal here to update the UI safely
            self.ui.contact_detect_indicator.setStyleSheet("background-color: orange;")
        else:
            self.ui.contact_detect_indicator.setStyleSheet("")

if __name__ == "__main__":
    rospy.init_node('qt_gui')
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())
