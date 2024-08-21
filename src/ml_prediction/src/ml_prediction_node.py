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

        # Initialize storage for initial predictions and deviation
        self.initial_fx_predictions = []
        self.initial_fy_predictions = []
        self.initial_fz_predictions = []
        self.deviation_fx = 0.0
        self.deviation_fy = 0.0
        self.deviation_fz = 0.0
        self.deviation_calculated = False

        dir_path = os.path.dirname(os.path.realpath(__file__))
        normalization_position_params_path = os.path.join(dir_path, 'Savings', 'normalization_position_params.pkl')
        normalization_force_params_path = os.path.join(dir_path, 'Savings', 'normalization_force_params.pkl')

        # Load normalization parameters for position prediction
        with open(normalization_position_params_path, 'rb') as file:
            self.norm_pos_params = pickle.load(file)

        # Load normalization parameters for force prediction
        with open(normalization_force_params_path, 'rb') as file:
            self.norm_force_params = pickle.load(file)

        # Use absolute paths for model files as well
        lstm_classification_model_path = os.path.join(dir_path, 'Savings', 'lstm_classification_model.h5')
        lstm_multiclassification_model_path = os.path.join(dir_path, 'Savings', 'lstm_multiclassification_model.h5')
        mlp_classification_model_path = os.path.join(dir_path, 'Savings', 'mlp_classification_model.h5')
        # normalforce_model_path = os.path.join(dir_path, 'Savings', 'normalforce_model.h5')
        normalforce_model_path = os.path.join(dir_path, 'Savings', 'normalforce_model.h5')
        shearforce_model_path = os.path.join(dir_path, 'Savings', 'shearforce_model.h5')

        # Load machine learning models
        self.lstm_classification_model = tf.keras.models.load_model(lstm_classification_model_path)
        self.lstm_multiclassification_model = tf.keras.models.load_model(lstm_multiclassification_model_path)
        self.mlp_classification_model = tf.keras.models.load_model(mlp_classification_model_path)
        self.normalforce_model = tf.keras.models.load_model(normalforce_model_path)
        self.shearforce_model = tf.keras.models.load_model(shearforce_model_path)

        # Subscriber to processed sensor data
        rospy.Subscriber(f'processed_sensor_data_{sensor_id}', Float32MultiArray, self.processed_data_callback)

        # Publisher for prediction results
        self.position_publisher = rospy.Publisher(f'prediction_position_{sensor_id}', Int32, queue_size=10)
        self.fz_publisher = rospy.Publisher(f'prediction_fz_{sensor_id}', Float32, queue_size=10)
        self.fx_publisher = rospy.Publisher(f'prediction_fx_{sensor_id}', Float32, queue_size=10)
        self.fy_publisher = rospy.Publisher(f'prediction_fy_{sensor_id}', Float32, queue_size=10)

        # Publisher for diagnostics
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        # self.diagnostics_timer = rospy.Timer(rospy.Duration(5), self.publish_periodic_diagnostics)
        self.diagnostic_status = {}     # Create a dictionary to hold diagnostic status for each sensor

        # Parameters
        self.classifier = "lstm"  # "mlp" or "lstm"
        self.touch_mode = "single"  # "single" or "multi"

        self.data_batch = []
        self.batch_size = 10

    def normalize(self, value, min_val, max_val):
        return 2 * ((value - min_val) / (max_val - min_val)) - 1

    def denormalize_force(self, f, f_min, f_max, scale_range=(-1, 1)):
        scale_min, scale_max = scale_range
        return ((f - scale_min) / (scale_max - scale_min)) * (f_max - f_min) + f_min

    def prediction_data(self, sensor_id, filtered_x, filtered_y, filtered_z):
        predicted_fx = 0.0
        predicted_fy = 0.0
        predicted_fz = 0.0  # Initialize with default value
        predicted_position_label = 0  # Initialize with default value

        try:
            self.data_batch.append([filtered_x, filtered_y, filtered_z])

            if len(self.data_batch) >= self.batch_size:
                batch = np.array(self.data_batch, dtype=np.float32)
                self.data_batch = []

                # Normalize the batch data for position prediction using position normalization parameters
                batch_norm_flux_pos = self.normalize_batch(batch, self.norm_pos_params)

                # Normalize the batch data for normal force prediction using normal flux normalization parameters
                batch_norm_flux_normal = self.normalize_batch(batch, self.norm_force_params['normal_flux'])

                # Normalize the batch data for shear force prediction using shear flux normalization parameters
                batch_norm_flux_shear = self.normalize_batch(batch[:, :2], self.norm_force_params['shear_flux'],
                                                             force_type='shear')  # Only Bx and By

                # Predict position using the batch
                if self.classifier == "mlp":
                    predicted_positions = self.mlp_classification_model.predict(batch_norm_flux_pos, verbose=0)
                    predicted_position_labels = np.argmax(predicted_positions, axis=1) + 1  # labels start from 1
                elif self.classifier == "lstm":
                    sequence_length = batch.shape[0]  # Use the batch size as sequence length
                    lstm_input_data = batch_norm_flux_pos.reshape(1, sequence_length, -1)
                    if self.touch_mode == "single":
                        predicted_positions = self.lstm_classification_model.predict(lstm_input_data, verbose=0)
                        predicted_position_labels = np.argmax(predicted_positions, axis=1) + 1
                    elif self.touch_mode == "multi":
                        predicted_positions = self.lstm_multiclassification_model.predict(lstm_input_data, verbose=0)
                        touch_threshold = 0.5  # Activation threshold for multi-touch case
                        predicted_position_labels = (predicted_positions >= touch_threshold).astype(int).flatten()

                # Use majority voting to determine the predicted position
                predicted_position_label = Counter(predicted_position_labels).most_common(1)[0][0]

                # Predict normal force (Fz) using the batch
                predicted_normalforce_output = self.normalforce_model.predict(batch_norm_flux_normal, verbose=0)
                predicted_fz = abs(float(
                    self.denormalize_force(predicted_normalforce_output.mean(),
                                           self.norm_force_params['force']['Fz_min'],
                                           self.norm_force_params['force']['Fz_max'],
                                           scale_range=(0, 1))))

                # Predict shear forces (Fx and Fy) using the batch
                predicted_shearforce_output = self.shearforce_model.predict(batch_norm_flux_shear, verbose=0)
                predicted_fx = self.denormalize_force(predicted_shearforce_output[:, 0].mean(),
                                                      self.norm_force_params['force']['Fx_min'],
                                                      self.norm_force_params['force']['Fx_max'],
                                                      scale_range=(-1, 1))
                predicted_fy = self.denormalize_force(predicted_shearforce_output[:, 1].mean(),
                                                      self.norm_force_params['force']['Fy_min'],
                                                      self.norm_force_params['force']['Fy_max'],
                                                      scale_range=(-1, 1))

                # Store the first five predictions for deviation calculation
                if len(self.initial_fx_predictions) < 5:
                    self.initial_fx_predictions.append(predicted_fx)
                    self.initial_fy_predictions.append(predicted_fy)
                    self.initial_fz_predictions.append(predicted_fz)
                elif not self.deviation_calculated:
                    # Calculate deviation after the first five predictions
                    self.deviation_fx = np.mean(self.initial_fx_predictions)
                    self.deviation_fy = np.mean(self.initial_fy_predictions)
                    self.deviation_fz = np.mean(self.initial_fz_predictions)
                    self.deviation_calculated = True

                # Apply deviation correction
                if self.deviation_calculated:
                    predicted_fx -= self.deviation_fx
                    predicted_fy -= self.deviation_fy
                    predicted_fz -= self.deviation_fz

                # Postprocessing
                if predicted_fz > 8:
                    predicted_fz *= 0.6
                elif self.touch_mode == "single" and predicted_position_label == 1:
                    predicted_fz *= 0.8
                elif self.touch_mode == "single" and predicted_position_label == 2:
                    predicted_fz *= 1.5
                elif self.touch_mode == "single" and predicted_position_label == 3:
                    predicted_fz *= 1.2
                elif self.touch_mode == "single" and predicted_position_label == 4:
                    predicted_fz *= 0.8

                # Publish the prediction results
                if not rospy.is_shutdown():
                    self.position_publisher.publish(predicted_position_label)
                    self.fz_publisher.publish(predicted_fz)
                    self.fx_publisher.publish(predicted_fx)
                    self.fy_publisher.publish(predicted_fy)

        finally:
            if not rospy.is_shutdown():
                self.publish_diagnostics(  # Publish diagnostics
                    True,
                    f"Machine learning prediction is running. Current prediction: {sensor_id}, Position: {predicted_position_label},"
                    f" Fz: {predicted_fz:.2f}N, Fx: {predicted_fx:.2f}N, Fy: {predicted_fy:.2f}N",
                    sensor_id, predicted_position_label, predicted_fz, predicted_fx, predicted_fy
                )

    def normalize_batch(self, batch, params, force_type='normal'):
        # Normalize the entire batch of data using provided parameters.
        batch_norm = np.zeros_like(batch)
        for i in range(batch.shape[0]):
            batch_norm[i, 0] = self.normalize(batch[i, 0], params['Bx_min'], params['Bx_max'])
            batch_norm[i, 1] = self.normalize(batch[i, 1], params['By_min'], params['By_max'])
            if force_type == 'normal':
                batch_norm[i, 2] = self.normalize(batch[i, 2], params['Bz_min'], params['Bz_max'])
        return batch_norm

    def processed_data_callback(self, msg):
        global subscription_count, start_time

        filtered_data = list(msg.data)
        self.prediction_data(self.sensor_id, *filtered_data)

        # elapsed_time = time.time() - start_time
        # subscription_count[sensor_id] += 1  # Update subscription count

    def publish_diagnostics(self, status, message, sensor_id=None, predicted_position=None, predicted_fz=0.0,
                            predicted_fx=0.0, predicted_fy=0.0):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()

        if sensor_id not in self.diagnostic_status:
            self.diagnostic_status[sensor_id] = DiagnosticStatus()
            self.diagnostic_status[sensor_id].name = f"MLLearningNode: {sensor_id}"
            self.diagnostic_status[sensor_id].hardware_id = f"MLLearningNode: {sensor_id}"

        status_msg = self.diagnostic_status[sensor_id]
        status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.ERROR
        status_msg.message = message

        if predicted_fz is not None:  # Convert predicted_fz to float if it's not None
            try:
                predicted_fz = float(predicted_fz)
            except ValueError:
                rospy.logerr(f"Failed to convert predicted_fz to float: {predicted_fz}")
                predicted_fz = None

        if predicted_fx is not None:  # Convert predicted_fx to float if it's not None
            try:
                predicted_fx = float(predicted_fx)
            except ValueError:
                rospy.logerr(f"Failed to convert predicted_fx to float: {predicted_fx}")
                predicted_fx = None

        if predicted_fy is not None:  # Convert predicted_fy to float if it's not None
            try:
                predicted_fy = float(predicted_fy)
            except ValueError:
                rospy.logerr(f"Failed to convert predicted_fy to float: {predicted_fy}")
                predicted_fy = None

        status_msg.values = [
            KeyValue("Sensor ID", sensor_id),
            KeyValue("Predicted Position", str(predicted_position)),
            KeyValue("Predicted Fz", f"{predicted_fz:.2f}N" if predicted_fz is not None else "N/A"),
            KeyValue("Predicted Fx", f"{predicted_fx:.2f}N" if predicted_fx is not None else "N/A"),
            KeyValue("Predicted Fy", f"{predicted_fy:.2f}N" if predicted_fy is not None else "N/A")
        ]

        diag_msg.status.append(status_msg)
        self.diag_pub.publish(diag_msg)

    def publish_periodic_diagnostics(self, event):
        global subscription_count, prediction_count, start_time

        # elapsed_time = time.time() - start_time

        for sensor_id, status_msg in self.diagnostic_status.items():
            predicted_position = None
            predicted_fz = 0.0
            predicted_fx = 0.0
            predicted_fy = 0.0

            if len(status_msg.values) > 1:
                predicted_position = status_msg.values[1].value
            if len(status_msg.values) > 2:
                predicted_fz = status_msg.values[2].value
                try:
                    predicted_fz = float(predicted_fz.replace("N", "").strip())
                except ValueError:
                    predicted_fz = None
            if len(status_msg.values) > 3:
                predicted_fx = status_msg.values[3].value
                try:
                    predicted_fx = float(predicted_fx.replace("N", "").strip())
                except ValueError:
                    predicted_fx = None
            if len(status_msg.values) > 4:
                predicted_fy = status_msg.values[4].value
                try:
                    predicted_fy = float(predicted_fy.replace("N", "").strip())
                except ValueError:
                    predicted_fy = None

            # prediction_rate = prediction_count[sensor_id] / elapsed_time
            # rospy.loginfo(f"Prediction rate for {sensor_id}: {prediction_rate:.2f} Hz")
            # subscription_rate = subscription_count[sensor_id] / elapsed_time
            # rospy.loginfo(f"Subscription rate for {sensor_id}: {subscription_rate:.2f} Hz")

            self.publish_diagnostics(
                True,
                f"Machine learning prediction is running for sensor {sensor_id}.",
                sensor_id=sensor_id,
                predicted_position=predicted_position,
                predicted_fz=predicted_fz,
                predicted_fx=predicted_fx,
                predicted_fy=predicted_fy
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

