#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import numpy as np

class SignalChangeDetector:
    def __init__(self, sensor_id, window_size=10, change_threshold=0.3):
        self.sensor_id = sensor_id
        self.window_size = window_size
        self.change_threshold = change_threshold
        self.data_window = {'x': [], 'y': [], 'z': []}
        self.change_flag = {'x': False, 'y': False, 'z': False}
        self.processed_sensor_data_subscriber = rospy.Subscriber('processed_sensor_data', String, self.callback)
        self.release_finish_subscriber = rospy.Subscriber('release_finish', String, self.release_finish_callback)
        self.stop_all_subscriber = rospy.Subscriber('stop_all', String, self.stop_all_callback)
        self.publisher = rospy.Publisher('contact_detect', String, queue_size=10)

    def callback(self, msg):
        data = json.loads(msg.data)
        if data['sensor_id'] == self.sensor_id:
            self.process_data(data['filtered_data'])

    def release_finish_callback(self, data):
        data_json = json.loads(data.data)
        if data_json.get('release_finish_flag'):
            rospy.loginfo('Release finished, initializing contact flag...')
            self.publisher.publish(json.dumps({'sensor_id': self.sensor_id, 'change_flag': False}))
            self.reset_data_window()# Consider resetting data and flags if necessary
            self.reset_change_flags()

    def stop_all_callback(self, data):
        data_json = json.loads(data.data)
        if data_json.get('stop_all_flag'):
            rospy.loginfo('Stopping...')
            self.publisher.publish(json.dumps({'sensor_id': self.sensor_id, 'change_flag': False}))
            self.reset_data_window()# Consider resetting data and flags if necessary
            self.reset_change_flags()

    def process_data(self, filtered_data):
        for axis, value in zip(['x', 'y', 'z'], filtered_data):
            self.data_window[axis].append(value)
            if len(self.data_window[axis]) > self.window_size:
                self.data_window[axis].pop(0)
                change_detected, max_change = self.analyze_change(self.data_window[axis])
                if change_detected:
                    rospy.loginfo(f"Significant change ({max_change}) detected on {axis} axis for {self.sensor_id}.")
                    self.change_flag[axis] = True
            else:
                rospy.loginfo("Dectecting signal changes...")

        # if all(self.change_flag.values()): # if changes in all axis are detected
        if list(self.change_flag.values()).count(True) >= 2:  # If changes are detected in at least two axes
            rospy.loginfo(f"Contact has been detected on {self.sensor_id}.")
            self.publisher.publish(json.dumps({'sensor_id': self.sensor_id, 'change_flag': True}))
            self.reset_data_window()    # Optionally reset flags and data windows to avoid repeated triggers
            self.reset_change_flags()

    def analyze_change(self, data):
        # Simple first derivative calculation (difference)
        change_flag = False
        differences = np.diff(data)
        max_change = np.max(np.abs(differences))
        if max_change > self.change_threshold:
            change_flag = True
        return change_flag, max_change

    def reset_change_flags(self):
        self.change_flag = {'x': False, 'y': False, 'z': False}

    def reset_data_window(self):
        self.data_window = {'x': [], 'y': [], 'z': []}


if __name__ == '__main__':
    rospy.init_node('signal_change_detection_node')
    sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs
    detectors = [SignalChangeDetector(sensor_id) for sensor_id in sensor_ids]
    rospy.spin()


