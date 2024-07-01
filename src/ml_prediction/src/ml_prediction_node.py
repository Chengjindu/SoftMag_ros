#!/usr/bin/env python3
import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # Suppress TensorFlow logging
import tensorflow as tf
tf.get_logger().setLevel('ERROR')
import logging
logging.getLogger('tensorflow').setLevel(logging.ERROR)

import rospy
from std_msgs.msg import Float32MultiArray, Int32, Float32
import json
import numpy as np
import pickle
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from concurrent.futures import ThreadPoolExecutor
from collections import Counter
import time

# # Global counters and timestamps for measuring rates
# subscription_count = {}
# prediction_count = {}
# start_time = time.time()

class MLLearningNode:
    def __init__(self, sensor_id):
        global subscription_count, prediction_count

        self.sensor_id = sensor_id

        # subscription_count[sensor_id] = 0       # Initialize counters
        # prediction_count[sensor_id] = 0

        # Determine the directory in which this script is located
        dir_path = os.path.dirname(os.path.realpath(__file__))
        normalization_params_path = os.path.join(dir_path, 'Savings', 'normalization_params.pkl')

        # Load normalization parameters
        with open(normalization_params_path, 'rb') as file:
            self.norm_params = pickle.load(file)

        # Use absolute paths for model files as well
        lstm_classification_model_path = os.path.join(dir_path, 'Savings', 'lstm_classification_model.h5')
        lstm_multiclassification_model_path = os.path.join(dir_path, 'Savings', 'lstm_multiclassification_model.h5')
        mlp_classification_model_path = os.path.join(dir_path, 'Savings', 'mlp_classification_model.h5')
        regression_model_path = os.path.join(dir_path, 'Savings', 'regression_model.h5')

        # Load machine learning models
        self.lstm_classification_model = tf.keras.models.load_model(lstm_classification_model_path)
        self.lstm_multiclassification_model = tf.keras.models.load_model(lstm_multiclassification_model_path)
        self.mlp_classification_model = tf.keras.models.load_model(mlp_classification_model_path)
        self.regression_model = tf.keras.models.load_model(regression_model_path)

        # Subscriber to processed sensor data
        rospy.Subscriber(f'processed_sensor_data_{sensor_id}', Float32MultiArray, self.processed_data_callback)

        # Publisher for prediction results
        self.position_publisher = rospy.Publisher(f'prediction_position_{sensor_id}', Int32, queue_size=10)
        self.force_publisher = rospy.Publisher(f'prediction_force_{sensor_id}', Float32, queue_size=10)

        # Publisher for diagnostics
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.diagnostics_timer = rospy.Timer(rospy.Duration(5), self.publish_periodic_diagnostics)
        self.diagnostic_status = {}     # Create a dictionary to hold diagnostic status for each sensor

        # Parameters
        self.classifier = "lstm"  # "mlp" or "lstm"
        self.touch_mode = "single"  # "single" or "multi"

        self.data_batch = []
        self.batch_size = 10

    def normalize(self, value, min_val, max_val):
        return 2 * ((value - min_val) / (max_val - min_val)) - 1

    def denormalize_force(self, f, f_min, f_max):
        return f * (f_max - f_min) + f_min

    def prediction_data(self, sensor_id, filtered_x, filtered_y, filtered_z):
        predicted_force = 0.0  # Initialize with default value
        predicted_position_label = 0  # Initialize with default value

        try:
            self.data_batch.append([filtered_x, filtered_y, filtered_z])

            if len(self.data_batch) >= self.batch_size:
                batch = np.array(self.data_batch, dtype=np.float32)
                self.data_batch = []

                # Normalize the batch data
                batch_norm = self.normalize_batch(batch)

                # Predict position using the batch
                if self.classifier == "mlp":
                    predicted_positions = self.mlp_classification_model.predict(batch_norm, verbose=0)
                    predicted_position_labels = np.argmax(predicted_positions,
                                                          axis=1) + 1  # Assuming labels start from 1
                elif self.classifier == "lstm":
                    sequence_length = batch.shape[0]  # Use the batch size as sequence length
                    lstm_input_data = batch_norm.reshape(1, sequence_length, -1)
                    if self.touch_mode == "single":
                        predicted_positions = self.lstm_classification_model.predict(lstm_input_data, verbose=0)
                        predicted_position_labels = np.argmax(predicted_positions,
                                                              axis=1) + 1  # Assuming labels start from 1
                    elif self.touch_mode == "multi":
                        predicted_positions = self.lstm_multiclassification_model.predict(lstm_input_data, verbose=0)
                        touch_threshold = 0.5  # Activation threshold for multi-touch case
                        predicted_position_labels = (predicted_positions >= touch_threshold).astype(int).flatten()

                # Use majority voting to determine the predicted position
                predicted_position_label = Counter(predicted_position_labels).most_common(1)[0][0]

                # Predict force using the batch
                predicted_force_output = self.regression_model.predict(batch_norm, verbose=0)
                predicted_force = abs(float(
                    self.denormalize_force(predicted_force_output.mean(), self.norm_params['f_min'],
                                           self.norm_params['f_max'])))

                # Example postprocessing, adjust as needed
                if predicted_force > 8:
                    predicted_force *= 0.6
                elif self.touch_mode == "single" and predicted_position_label == 1:
                    predicted_force *= 0.8
                elif self.touch_mode == "single" and predicted_position_label == 2:
                    predicted_force *= 1.5
                elif self.touch_mode == "single" and predicted_position_label == 3:
                    predicted_force *= 1.2
                elif self.touch_mode == "single" and predicted_position_label == 4:
                    predicted_force *= 0.8

                # Publish the prediction results
                if not rospy.is_shutdown():
                    self.position_publisher.publish(predicted_position_label)
                    self.force_publisher.publish(predicted_force)

                # prediction_count[sensor_id] += 1    # Update prediction count

        finally:
            if not rospy.is_shutdown():
                self.publish_diagnostics(  # Publish diagnostics
                    True,
                    f"Machine learning prediction is running. Current prediction: {sensor_id}, Position: {predicted_position_label}, Force: {predicted_force:.2f}N",
                    sensor_id, predicted_position_label, predicted_force
                )

    def normalize_batch(self, batch):
        # Normalize the entire batch of data.
        batch_norm = np.zeros_like(batch)
        for i in range(batch.shape[0]):
            batch_norm[i, 0] = self.normalize(batch[i, 0], self.norm_params['Bx_min'], self.norm_params['Bx_max'])
            batch_norm[i, 1] = self.normalize(batch[i, 1], self.norm_params['By_min'], self.norm_params['By_max'])
            batch_norm[i, 2] = self.normalize(batch[i, 2], self.norm_params['Bz_min'], self.norm_params['Bz_max'])
        return batch_norm

    def processed_data_callback(self, msg):
        global subscription_count, start_time

        filtered_data = list(msg.data)
        self.prediction_data(self.sensor_id, *filtered_data)

        # elapsed_time = time.time() - start_time
        # subscription_count[sensor_id] += 1  # Update subscription count

    def publish_diagnostics(self, status, message, sensor_id = None, predicted_position = None, predicted_force = 0.0):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()

        if sensor_id not in self.diagnostic_status:
            self.diagnostic_status[sensor_id] = DiagnosticStatus()
            self.diagnostic_status[sensor_id].name = f"MLLearningNode: {sensor_id}"
            self.diagnostic_status[sensor_id].hardware_id = f"MLLearningNode: {sensor_id}"

        status_msg = self.diagnostic_status[sensor_id]
        status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.ERROR
        status_msg.message = message

        if predicted_force is not None: # Convert predicted_force to float if it's not None
            try:
                predicted_force = float(predicted_force)
            except ValueError:
                rospy.logerr(f"Failed to convert predicted_force to float: {predicted_force}")
                predicted_force = None

        status_msg.values = [
            KeyValue("Sensor ID", sensor_id),
            KeyValue("Predicted Position", str(predicted_position)),
            KeyValue("Predicted Force", f"{predicted_force:.2f}N" if predicted_force is not None else "N/A")
        ]

        diag_msg.status.append(status_msg)
        self.diag_pub.publish(diag_msg)

    def publish_periodic_diagnostics(self, event):
        global subscription_count, prediction_count, start_time

        # elapsed_time = time.time() - start_time

        for sensor_id, status_msg in self.diagnostic_status.items():
            predicted_position = None
            predicted_force = 0.0
            if len(status_msg.values) > 1:
                predicted_position = status_msg.values[1].value
            if len(status_msg.values) > 2:
                predicted_force = status_msg.values[2].value
                try:
                    predicted_force = float(predicted_force.replace("N", "").strip())
                except ValueError:
                    predicted_force = None

            # prediction_rate = prediction_count[sensor_id] / elapsed_time
            # rospy.loginfo(f"Prediction rate for {sensor_id}: {prediction_rate:.2f} Hz")
            # subscription_rate = subscription_count[sensor_id] / elapsed_time
            # rospy.loginfo(f"Subscription rate for {sensor_id}: {subscription_rate:.2f} Hz")

            self.publish_diagnostics(
                True,
                f"Machine learning prediction is running for sensor {sensor_id}.",
                sensor_id=sensor_id,
                predicted_position = predicted_position,
                predicted_force=predicted_force
            )


if __name__ == '__main__':
    try:
        rospy.init_node('ml_prediction_node')  # Initialize ROS node
        sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs

        # Use ThreadPoolExecutor for parallel processing
        with ThreadPoolExecutor(max_workers=len(sensor_ids)) as executor:
            for sensor_id in sensor_ids:
                executor.submit(MLLearningNode, sensor_id)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

