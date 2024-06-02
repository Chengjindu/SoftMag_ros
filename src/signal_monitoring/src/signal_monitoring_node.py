#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SignalChangeDetector:
    def __init__(self, sensor_id, window_size=10, change_threshold=0.3):
        self.sensor_id = sensor_id
        self.window_size = window_size
        self.change_threshold = change_threshold
        self.contact_detected = False
        self.contact_trigger = False
        self.current_mode = "Sensing"

        self.data_window = {'x': [], 'y': [], 'z': []}
        self.change_flag = {'x': False, 'y': False, 'z': False}
        self.processed_sensor_data_subscriber = rospy.Subscriber('processed_sensor_data', String, self.callback)
        self.release_finish_subscriber = rospy.Subscriber('release_finish', String, self.release_finish_callback)
        self.stop_all_subscriber = rospy.Subscriber('stop_all', String, self.stop_all_callback)
        self.contact_trigger_publisher = rospy.Publisher('contact_trigger', String, queue_size=10)
        self.contact_detected_publisher = rospy.Publisher('contact_detect', String, queue_size=10)
        self.mode_subscriber = rospy.Subscriber('mode_change', String, self.mode_callback)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    def callback(self, msg):
        data = json.loads(msg.data)
        if data['sensor_id'] == self.sensor_id:
            self.contact_detection(data['filtered_data'])

    def release_finish_callback(self, data):
        data_json = json.loads(data.data)
        if data_json.get('release_finish_flag'):
            self.contact_trigger = False
            self.contact_trigger_publisher.publish(json.dumps({'change_flag': False}))
            self.reset_data_window()  # Consider resetting data and flags if necessary
            self.reset_change_flags()
            self.publish_diagnostics(True, "Release finished, initializing contact flag...")

    def stop_all_callback(self, data):
        data_json = json.loads(data.data)
        if data_json.get('stop_all_flag'):
            self.contact_trigger = False
            self.contact_trigger_publisher.publish(json.dumps({'change_flag': False}))
            self.reset_data_window()  # Consider resetting data and flags if necessary
            self.reset_change_flags()
            self.publish_diagnostics(True, "Stopping...")

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            self.contact_trigger = False
            self.contact_trigger_publisher.publish(json.dumps({'change_flag': False}))
            self.reset_data_window()  # Consider resetting data and flags if necessary
            self.reset_change_flags()
            self.publish_diagnostics(True, f"Mode changed to {new_mode}, initializing contact flag...")
            self.current_mode = new_mode

    def contact_detection(self, filtered_data):
        for axis, value in zip(['x', 'y', 'z'], filtered_data):
            self.data_window[axis].append(value)
            if len(self.data_window[axis]) > self.window_size:
                self.data_window[axis].pop(0)
                change_detected, max_change = self.analyze_change(self.data_window[axis])
                if change_detected:
                    self.change_flag[axis] = True
                    self.publish_diagnostics(True,
                                             f"Significant change ({max_change}) detected on {axis} axis for {self.sensor_id}.")
            else:
                self.publish_diagnostics(True, "Detecting signal changes...")

        # if all(self.change_flag.values()): # if changes in all axis are detected
        if list(self.change_flag.values()).count(True) >= 2:  # If changes are detected in at least two axes
            if not self.contact_detected:   # when no contact is detected, or at the first run
                if not self.contact_trigger:    # when contact_trigger is set to be false (first run or reinitialized)
                    self.contact_trigger = True
                    self.contact_trigger_publisher.publish(json.dumps({'change_flag': True}))
                self.contact_detected = True    # Set status as contact detected
                # self.contact_detected_publisher.publish(json.dumps({'sensor_id': self.sensor_id, 'contact_detected': True}))
                self.publish_diagnostics(True, f"Contact has been detected on {self.sensor_id}.")
            else:   # when there is contact detected in the last run, and changes are detected in at least two axes for this run
                self.contact_detected = False
                # self.contact_detected_publisher.publish(json.dumps({'sensor_id': self.sensor_id, 'contact_detected': False}))
                self.publish_diagnostics(True, f"Contact has been removed on {self.sensor_id}.")
            self.reset_data_window()  # Optionally reset flags and data windows to avoid repeated triggers
            self.reset_change_flags()

        # Calculate the Euclidean norm of the sensing data vector
        norm = np.linalg.norm(filtered_data)
        # Perform the additional significant check
        if self.contact_detected:   # if contact is detected last time
            if norm < 0.4:  # Lower threshold check
                self.contact_detected = False
                self.contact_detected_publisher.publish(
                    json.dumps({'sensor_id': self.sensor_id, 'contact_detected': False}))
                self.publish_diagnostics(True, f"Contact has been removed on {self.sensor_id}.")
            else:
                self.publish_diagnostics(True, "Contact ongoing...")
                # self.contact_detected_publisher.publish(
                #     json.dumps({'sensor_id': self.sensor_id, 'contact_detected': True}))
        else:
            if norm > 0.8:  # Upper threshold check
                self.contact_detected = True
                self.contact_detected_publisher.publish(
                    json.dumps({'sensor_id': self.sensor_id, 'contact_detected': True}))
                self.publish_diagnostics(True, f"Contact has been detected on {self.sensor_id}.")
            else:
                self.publish_diagnostics(True, "No significant changes detected.")
                # self.contact_detected_publisher.publish(
                #     json.dumps({'sensor_id': self.sensor_id, 'contact_detected': False}))

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

    def publish_diagnostics(self, status, message):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        status_msg = DiagnosticStatus()
        status_msg.name = f"SignalChangeDetector: {self.sensor_id}"
        status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.WARN
        status_msg.message = message
        status_msg.hardware_id = self.sensor_id
        status_msg.values = [
            KeyValue("Sensor ID", self.sensor_id),
            KeyValue("Window Size", str(self.window_size)),
            KeyValue("Change Threshold", str(self.change_threshold)),
            KeyValue("Contact Detected", str(self.contact_detected)),
            KeyValue("Contact Triggered", str(self.contact_trigger))
        ]
        diag_msg.status.append(status_msg)
        self.diag_pub.publish(diag_msg)


if __name__ == '__main__':
    rospy.init_node('signal_change_detection_node')
    sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs
    detectors = [SignalChangeDetector(sensor_id) for sensor_id in sensor_ids]
    rospy.spin()


