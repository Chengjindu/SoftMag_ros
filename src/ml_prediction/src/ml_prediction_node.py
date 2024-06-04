#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import numpy as np
import tensorflow as tf
import pickle
import sys
import os
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class MLLearningNode:
    def __init__(self):
        # Suppress TensorFlow logging
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
        tf.get_logger().setLevel('ERROR')

        # Initialize the ROS node
        rospy.init_node('ml_learning_node', anonymous=True)

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
        rospy.Subscriber('processed_sensor_data', String, self.processed_data_callback)

        # Publisher for prediction results
        self.prediction_publisher = rospy.Publisher('prediction_results', String, queue_size=10)

        # Publisher for diagnostics
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.diagnostics_timer = rospy.Timer(rospy.Duration(5), self.publish_periodic_diagnostics)
        # Create a dictionary to hold diagnostic status for each sensor
        self.diagnostic_status = {}

        # Parameters
        self.classifier = "lstm"  # "mlp" or "lstm"
        self.touch_mode = "single"  # "single" or "multi"

    def normalize(self, value, min_val, max_val):
        return 2 * ((value - min_val) / (max_val - min_val)) - 1

    def denormalize_force(self, f, f_min, f_max):
        return f * (f_max - f_min) + f_min

    def process_data(self, sensor_id, filtered_x, filtered_y, filtered_z):
        # Suppress stdout and stderr during prediction
        global predicted_position_label
        sys.stdout = open(os.devnull, 'w')
        sys.stderr = open(os.devnull, 'w')
        try:
            # Normalize the sensor data
            filtered_x_norm = self.normalize(filtered_x, self.norm_params['Bx_min'], self.norm_params['Bx_max'])
            filtered_y_norm = self.normalize(filtered_y, self.norm_params['By_min'], self.norm_params['By_max'])
            filtered_z_norm = self.normalize(filtered_z, self.norm_params['Bz_min'], self.norm_params['Bz_max'])
            normalized_data_point = np.array([filtered_x_norm, filtered_y_norm, filtered_z_norm], dtype=np.float32).reshape(
                1, -1)  # Reshape for model input

            # Predict position
            if self.classifier == "mlp":
                predicted_position = self.mlp_classification_model.predict(normalized_data_point)
                predicted_position_label = np.argmax(predicted_position) + 1  # Assuming labels start from 1
            elif self.classifier == "lstm":
                # Assuming an LSTM model expecting sequences; it might need to maintain a sequence buffer as a class attribute
                # Here use a dummy sequence buffer filled with the current data point
                sequence_length = 10  # Sequence length
                lstm_input_data = np.tile(normalized_data_point, (sequence_length, 1)).reshape(1, sequence_length,
                                                                                               -1)  # Reshape for LSTM model input
                if self.touch_mode == "single":
                    predicted_position = self.lstm_classification_model.predict(lstm_input_data)
                    predicted_position_label = np.argmax(predicted_position) + 1  # Assuming labels start from 1
                elif self.touch_mode == "multi":
                    predicted_position = self.lstm_multiclassification_model.predict(lstm_input_data)
                    touch_threshold = 0.5  # activation threshold for multi-touch case
                    predicted_position_label = (predicted_position >= touch_threshold).astype(int).flatten()

            # Predict force
            predicted_force_output = self.regression_model.predict(normalized_data_point)
            predicted_force = abs(
                float(self.denormalize_force(predicted_force_output, self.norm_params['f_min'], self.norm_params['f_max'])))

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
            prediction_result = {
                'sensor_id': sensor_id,
                'position': int(predicted_position_label),
                'force': float(predicted_force)
            }
            self.prediction_publisher.publish(json.dumps(prediction_result))

        finally:
            # Restore stdout and stderr
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__

        # Publish diagnostics
        self.publish_diagnostics(
            True,
            f"Machine learning prediction is running. Current prediction: {sensor_id}, Position: {predicted_position_label}, Force: {predicted_force:.2f}N",
            sensor_id, predicted_position_label, predicted_force
        )

    def processed_data_callback(self, msg):
        data = json.loads(msg.data)
        sensor_id = data['sensor_id']
        filtered_data = data['filtered_data']

        # Call the process_data function with the filtered sensor data
        self.process_data(sensor_id, *filtered_data)

    def publish_diagnostics(self, status, message, sensor_id=None, predicted_position=None, predicted_force=None):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()

        if sensor_id not in self.diagnostic_status:
            self.diagnostic_status[sensor_id] = DiagnosticStatus()
            self.diagnostic_status[sensor_id].name = f"MLLearningNode: {sensor_id}"
            self.diagnostic_status[sensor_id].hardware_id = f"MLLearningNode: {sensor_id}"

        status_msg = self.diagnostic_status[sensor_id]
        status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.ERROR
        status_msg.message = message

        # # Convert predicted_force to float if it's not None
        # if predicted_force is not None:
        #     try:
        #         predicted_force = float(predicted_force)
        #     except ValueError:
        #         rospy.logerr(f"Failed to convert predicted_force to float: {predicted_force}")
        #         predicted_force = 0.0  # Default value or handle appropriately

        status_msg.values = [
            KeyValue("Sensor ID", sensor_id),
            KeyValue("Predicted Position", str(predicted_position)),
            KeyValue("Predicted Force", f"{predicted_force:.2f}N" if predicted_force is not None else "N/A")
        ]

        diag_msg.status.append(status_msg)
        self.diag_pub.publish(diag_msg)

    def publish_periodic_diagnostics(self, event):
        for sensor_id, status_msg in self.diagnostic_status.items():
            self.publish_diagnostics(
                True,
                f"Machine learning prediction is running for sensor {sensor_id}.",
                sensor_id=sensor_id,
                predicted_position=status_msg.values[1].value if len(status_msg.values) > 1 else None,
                predicted_force=status_msg.values[2].value if len(status_msg.values) > 2 else None
            )


if __name__ == '__main__':
    try:
        ml_node = MLLearningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

