#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Int32, Float32, String
from std_srvs.srv import Trigger, TriggerResponse
import json
import time
import datetime
import numpy as np
from scipy.interpolate import interp1d
import subprocess
import signal
import sys
import os

def get_config_param(param):
    # Adjust the path to locate start_config.json
    config_file = os.path.join(os.path.dirname(__file__), '../../../start_config.json')
    result = subprocess.run(['jq', '-r', f'.{param}', config_file], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.strip()
    else:
        raise Exception(f"Failed to get config param '{param}': {result.stderr}")

class DataRecordingNode:
    def __init__(self):
        rospy.init_node('data_recording_node')

        self.default_mode = get_config_param('default_pi_mode')
        self.current_mode = None
        self.subscribers = {}
        self.is_recording = False
        self.data_buffer = {
            'sensor_data_S1': [],
            'sensor_data_S2': [],
            'prediction_position_S1': [],
            'prediction_position_S2': [],
            'prediction_force_S1': [],
            'prediction_force_S2': [],
            'pressure_reading1': [],
            'pressure_reading2': []
        }

        # Service to start/stop recording
        self.recording_service = rospy.Service('/record_data', Trigger, self.handle_recording)

        # Set the output directory
        self.output_directory = '/home/chengjin/Projects/SoftMag/ros_workspace/src/data_recording/src'
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

        self.file = None
        self.start_time = None

        # Set default mode and update subscribers accordingly
        self.set_default_mode(self.default_mode)

        signal.signal(signal.SIGINT, self.signal_handler)   # Register signal handler

    def set_default_mode(self, mode):
        self.current_mode = mode
        self.update_mode_parameters(mode)

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            self.update_mode_parameters(new_mode)

    def update_mode_parameters(self, mode):
        self.unsubscribe_all()

        if mode == "sensor":
            self.subscribers['sensor_data_S1'] = rospy.Subscriber('/processed_sensor_data_S1', Float32MultiArray, self.sensor_data_callback, 'sensor_data_S1')
            self.subscribers['sensor_data_S2'] = rospy.Subscriber('/processed_sensor_data_S2', Float32MultiArray, self.sensor_data_callback, 'sensor_data_S2')
            self.subscribers['prediction_position_S1'] = rospy.Subscriber('/prediction_position_S1', Int32, self.prediction_callback, 'prediction_position_S1')
            self.subscribers['prediction_position_S2'] = rospy.Subscriber('/prediction_position_S2', Int32, self.prediction_callback, 'prediction_position_S2')
            self.subscribers['prediction_force_S1'] = rospy.Subscriber('/prediction_force_S1', Float32, self.prediction_callback, 'prediction_force_S1')
            self.subscribers['prediction_force_S2'] = rospy.Subscriber('/prediction_force_S2', Float32, self.prediction_callback, 'prediction_force_S2')
        elif mode in ["gripper_automatic", "gripper_testing"]:
            self.subscribers['sensor_data_S1'] = rospy.Subscriber('/processed_sensor_data_S1', Float32MultiArray, self.sensor_data_callback, 'sensor_data_S1')
            self.subscribers['sensor_data_S2'] = rospy.Subscriber('/processed_sensor_data_S2', Float32MultiArray, self.sensor_data_callback, 'sensor_data_S2')
            self.subscribers['prediction_position_S1'] = rospy.Subscriber('/prediction_position_S1', Int32, self.prediction_callback, 'prediction_position_S1')
            self.subscribers['prediction_position_S2'] = rospy.Subscriber('/prediction_position_S2', Int32, self.prediction_callback, 'prediction_position_S2')
            self.subscribers['prediction_force_S1'] = rospy.Subscriber('/prediction_force_S1', Float32, self.prediction_callback, 'prediction_force_S1')
            self.subscribers['prediction_force_S2'] = rospy.Subscriber('/prediction_force_S2', Float32, self.prediction_callback, 'prediction_force_S2')
            self.subscribers['pressure_reading1'] = rospy.Subscriber('/pressure_reading1', Float32, self.pressure_callback, 'pressure_reading1')
            self.subscribers['pressure_reading2'] = rospy.Subscriber('/pressure_reading2', Float32, self.pressure_callback, 'pressure_reading2')

    def unsubscribe_all(self):
        for topic, subscriber in self.subscribers.items():
            subscriber.unregister()
        self.subscribers = {}

    def handle_recording(self, req):
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()
        return TriggerResponse(success=True, message="Recording toggled.")

    def start_recording(self):
        self.is_recording = True
        self.start_time = time.time()
        start_time_str = datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d_%H-%M-%S')
        self.file_path = os.path.join(self.output_directory, f'data_record_{start_time_str}.txt')
        self.file = open(self.file_path, 'w')
        rospy.loginfo(f"Started recording data. File path: {self.file_path}")

    def stop_recording(self):
        self.is_recording = False
        self.write_data_to_file()
        self.file.close()
        rospy.loginfo(f"Stopped recording data. File saved to: {self.file_path}")

    def sensor_data_callback(self, data, topic):
        if self.is_recording:
            timestamp = rospy.Time.now().to_sec()
            if isinstance(data.data, tuple) and len(data.data) >= 3:
                self.data_buffer[topic].append((timestamp, data.data))  # Append the whole tuple (x, y, z)
            else:
                self.data_buffer[topic].append((timestamp, (0, 0, 0)))  # Default to (0, 0, 0) if data is not valid

    def prediction_callback(self, data, topic):
        if self.is_recording:
            timestamp = rospy.Time.now().to_sec()
            self.data_buffer[topic].append((timestamp, data.data))

    def pressure_callback(self, data, topic):
        if self.is_recording:
            timestamp = rospy.Time.now().to_sec()
            self.data_buffer[topic].append((timestamp, data.data))

    def interpolate_data(self, timestamps, values, target_timestamps):
        if not timestamps or not values:
            rospy.loginfo("No data available for interpolation.")
            return [0] * len(target_timestamps)  # Return a list of zeros if there is no data

        rospy.loginfo(f"Interpolating data: {len(timestamps)} timestamps, {len(values)} values")
        if len(timestamps) < 2 or len(timestamps) != len(values):
            rospy.logwarn(
                f"Insufficient data for interpolation: {len(timestamps)} timestamps and {len(values)} values.")
            return [values[0]] * len(target_timestamps) if values else [0] * len(
                target_timestamps)  # Handle insufficient data
        interpolator = interp1d(timestamps, values, kind='linear', bounds_error=False, fill_value='extrapolate')
        return interpolator(target_timestamps)

    def write_data_to_file(self):
        if not any(self.data_buffer.values()):
            rospy.loginfo("No data to write.")
            return

        all_timestamps = sorted(set(
            timestamp for data_list in self.data_buffer.values() if data_list for timestamp, _ in data_list
        ))

        interpolated_data = {}
        for key, data_list in self.data_buffer.items():
            if self.current_mode == "sensor" and "pressure_reading" in key:
                continue  # Skip pressure reading data processing in sensor mode

            if not data_list:
                rospy.loginfo(f"No data for {key}, skipping interpolation.")
                interpolated_data[key] = {'x': [0] * len(all_timestamps), 'y': [0] * len(all_timestamps),
                                          'z': [0] * len(all_timestamps)} if 'sensor_data' in key else [0] * len(
                    all_timestamps)
                continue

            timestamps = [t for t, _ in data_list]
            if 'sensor_data' in key:
                values = [d for _, d in data_list]
                interpolated_data[key] = {
                    'x': self.interpolate_data(timestamps, [v[0] for v in values], all_timestamps),
                    'y': self.interpolate_data(timestamps, [v[1] for v in values], all_timestamps),
                    'z': self.interpolate_data(timestamps, [v[2] for v in values], all_timestamps)
                }
            else:
                values = [d for _, d in data_list]
                try:
                    interpolated_data[key] = self.interpolate_data(timestamps, values, all_timestamps)
                except Exception as e:
                    rospy.logerr(f"Failed to interpolate data for {key}: {e}")
                    interpolated_data[key] = [0] * len(all_timestamps)

        with open(self.file_path, 'w') as f:
            for i, timestamp in enumerate(all_timestamps):
                record = {"timestamp": timestamp}
                for key in interpolated_data:
                    if 'sensor_data' in key:
                        record[f"{key}_x"] = interpolated_data[key]['x'][i]
                        record[f"{key}_y"] = interpolated_data[key]['y'][i]
                        record[f"{key}_z"] = interpolated_data[key]['z'][i]
                    else:
                        record[key] = interpolated_data[key][i]
                f.write(json.dumps(record) + '\n')

        self.data_buffer = {key: [] for key in self.data_buffer}  # Clear buffer after writing

    def signal_handler(self, sig, frame):
        if self.is_recording:
            rospy.loginfo("SIGINT received. Stopping recording and saving data.")
            self.stop_recording()
        rospy.signal_shutdown("SIGINT received")
        sys.exit(0)

if __name__ == "__main__":
    recorder = DataRecordingNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass