#!/usr/bin/env python3
import rospy
from rospy import Time
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import json
from scipy.signal import cheby1, lfilter, lfilter_zi
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
import os
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Global variables for storing sensor data and filter configurations
sensor_data_lock = Lock()
sensor_data = {}
filters = {}
delay = None
data_stablized = {}
contact_detected = {}

def initialize_filters(sensor_ids, sampling_freq = 15, passband_freq = 3, order = 1, passband_ripple = 1):
    global filters, delay
    # passband_freq specifies the cutoff frequency for the filter
    # Frequencies below this value are allowed to pass, while those above are attenuated
    # order defines the filter's complexity and steepness of the roll-off
    # Higher order filters have a steeper drop-off but can introduce more delay and potential ringing artifacts
    # passband_ripple specifies the allowed ripple in the passband (in dB)
    # It defines how much the filter's response can deviate from flatness within the passband
    # Lower values result in a flatter response, but achieving this may require a higher order filter

    nyquist_freq = sampling_freq / 2    # The highest frequency that can be effectively captured by the filter without encountering aliasing.
    normalized_passband_freq = passband_freq / nyquist_freq     # Normalize the passband frequency with respect to the Nyquist frequency.
    # Normalization is necessary for the digital filter design, as it works with frequencies in a unit circle representation.

    # Design a Chebyshev type I low-pass filter with the given specifications.
    b, a = cheby1(N=order, rp=passband_ripple, Wn=normalized_passband_freq, btype='low')
    # b and a represent the numerator and denominator coefficients of the filter's transfer function, respectively.
    delay = 1.0 / sampling_freq

    for sensor_id in sensor_ids:
        zi = lfilter_zi(b, a)   # zi refers to the initial conditions for the filter's state.
        # When using lfilter, which stands for "linear filter," you're applying a linear, time-invariant filter to the data.
        filters[sensor_id] = {
            'b': b, 'a': a,
            'zi_x': zi * np.zeros(len(b) - 1),
            'zi_y': zi * np.zeros(len(b) - 1),
            'zi_z': zi * np.zeros(len(b) - 1)
        }

    return delay, filters

# Callback function for sensor_data topic
def sensor_data_callback(msg):
    global sensor_data  # Access the global variable to store sensor data

    parts = msg.data.split(',') # Split the incoming data string by commas
    if len(parts) < 4:
        rospy.logwarn(f"Received data in unexpected format: {msg.data}")
        return

    # Extract sensor ID and measurements from the split parts
    sensor_id, measurements = parts[0], [float(part) for part in parts[1:]]

    with sensor_data_lock:  # Store the measurements in the global dictionary using sensor_id as the key
        sensor_data[sensor_id] = measurements

def contact_detected_callback(msg):
    data = json.loads(msg.data)
    sensor_id = data['sensor_id']
    contact_detected[sensor_id] = data['contact_detected']

# Function to apply filter to the data
def filtering(data, filter_config):
    try:
        xMag, yMag, zMag = data
        b, a = filter_config['b'], filter_config['a']
        zi_x, zi_y, zi_z = filter_config['zi_x'], filter_config['zi_y'], filter_config['zi_z']
        filtered_x, zi_x = lfilter(b, a, [xMag], zi=zi_x)
        filtered_y, zi_y = lfilter(b, a, [yMag], zi=zi_y)
        filtered_z, zi_z = lfilter(b, a, [zMag], zi=zi_z)
        filter_config['zi_x'] = zi_x
        filter_config['zi_y'] = zi_y
        filter_config['zi_z'] = zi_z
        return filtered_x[0], filtered_y[0], filtered_z[0]
    except Exception as e:
        rospy.logwarn("Error filtering data: %s", e)
        return 0.0, 0.0, 0.0

# Process data for a single sensor
def process_sensor_data(sensor_id):
    global sensor_data, filters, delay, data_stablized, contact_detected

    # Initialize the contact_detected dictionary with default values for each sensor
    contact_detected[sensor_id] = False

    # Initialize state tracking dictionaries outside the loop
    Readout_count = 0
    initial_values = [None, None]  # List to hold the current and previous initial values
    data_stablized[sensor_id] = False
    message_printed = False
    all_stabilized_flag = False

    # Initialize buffer for recent data points and contact states
    deviation_update_buffer = {'x': [], 'y': [], 'z': []}
    contact_state_buffer = []
    deviation_update_window_size = 15  # Window size for recent data points
    contact_state_window_size = 15  # Window size for contact state tracking
    deviation_update_threshold = 0.1  # Threshold for Euclidean norm check
    delta_initial_x = 0
    delta_initial_y = 0
    delta_initial_z = 0

    while not rospy.is_shutdown():
        data = None
        with sensor_data_lock:
            if sensor_id in sensor_data and sensor_data[sensor_id] is not None:
                data = sensor_data.pop(sensor_id, None)

        if data is not None and len(data) == 3:
            try:
                filtered_x, filtered_y, filtered_z = filtering(data, filters[sensor_id])
                # Check if data has stabilized and initial_values is not set
                if not data_stablized[sensor_id] and initial_values[0] is None:
                    if not message_printed:
                        rospy.loginfo(f"Waiting readout for {sensor_id} to stabilize...")
                        message_printed = True
                    Readout_count += 1

                    if Readout_count >= 100:  # Assuming stabilization after 100 readings
                        rospy.loginfo(f"data stable for {sensor_id}.")
                        initial_values[0] = (
                        filtered_x, filtered_y, filtered_z)  # Set initial values for deviation elimination
                        initial_values[1] = initial_values[0]
                        data_stablized[sensor_id] = True

                if data_stablized[
                    sensor_id] and initial_values is not None:  # If data is stabilized, apply deviation elimination
                    # filtered_x = filtered_x - initial_values[0][0] - delta_initial_x
                    # filtered_y = filtered_y - initial_values[0][1] - delta_initial_y
                    # filtered_z = filtered_z - initial_values[0][2] - delta_initial_z
                    filtered_x = filtered_x - initial_values[0][0]
                    filtered_y = filtered_y - initial_values[0][1]
                    filtered_z = filtered_z - initial_values[0][2]

                    all_stabilized = all(data_stablized.values())  # Check if all values in the dictionary are True
                    if all_stabilized and not all_stabilized_flag:
                        rospy.loginfo("All sensor data stabilized.")
                        all_stabilized_flag = True

                    # Add current contact state to the buffer
                    contact_state_buffer.append(contact_detected[sensor_id])
                    if len(contact_state_buffer) > contact_state_window_size:
                        contact_state_buffer.pop(0)
                    # Check if the recent contact state buffer indicates no contact,
                    if contact_state_buffer.count(False) == contact_state_window_size:
                        # Check if the current data looks invalid
                        if abs(filtered_x) < 0.5 and abs(filtered_y) < 0.5 and abs(filtered_z) < 0.5:
                            # Add filtered data to the buffer
                            deviation_update_buffer['x'].append(filtered_x)
                            deviation_update_buffer['y'].append(filtered_y)
                            deviation_update_buffer['z'].append(filtered_z)
                            # Keep the buffer for updating deviation reference updated
                            if len(deviation_update_buffer['x']) > deviation_update_window_size:
                                deviation_update_buffer['x'].pop(0)
                                deviation_update_buffer['y'].pop(0)
                                deviation_update_buffer['z'].pop(0)

                        if len(deviation_update_buffer['x']) == deviation_update_window_size:  # Ensure buffer is full
                            avg_x = np.mean(deviation_update_buffer['x'])
                            avg_y = np.mean(deviation_update_buffer['y'])
                            avg_z = np.mean(deviation_update_buffer['z'])
                            norm = np.linalg.norm([avg_x, avg_y, avg_z])
                            if norm > deviation_update_threshold:   # Check if current deviation exceeds the update threshold
                                delta_initial_x = avg_x
                                delta_initial_y = avg_y
                                delta_initial_z = avg_z

                    # Prepare and publish the processed data
                    publish_data = {
                        'sensor_id': sensor_id,
                        'filtered_data': [filtered_x, filtered_y, filtered_z],
                        'stable_flag': all_stabilized  # Include the data_stablized flag
                    }
                    publisher.publish(json.dumps(publish_data))

                # Publish diagnostic message indicating success
                publish_diagnostics(sensor_id, True, "Data processing successful", filtered_data=(filtered_x, filtered_y, filtered_z))
                rospy.sleep(delay)

            except Exception as e:
                rospy.logerr(f"Error filtering data for sensor {sensor_id}: {e}")
                # Publish diagnostic message indicating failure
                publish_diagnostics(sensor_id, False, f"Error filtering data for sensor {sensor_id}: {e}")


def publish_diagnostics(sensor_id, status, message, filtered_data=None):
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = Time.now()  # Set the current time
    status_msg = DiagnosticStatus()
    status_msg.name = f"DataProcessingNode: {sensor_id}"
    status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.ERROR
    status_msg.message = message
    status_msg.hardware_id = sensor_id
    status_msg.values = [
        KeyValue("Sensor ID", sensor_id),
        KeyValue("Processing Status", "Success" if status else "Failure")
    ]

    if filtered_data:   # If filtered data is provided, add it to the diagnostics message
        status_msg.values.append(KeyValue("Filtered X", str(filtered_data[0])))
        status_msg.values.append(KeyValue("Filtered Y", str(filtered_data[1])))
        status_msg.values.append(KeyValue("Filtered Z", str(filtered_data[2])))

    diag_msg.status.append(status_msg)
    diag_pub.publish(diag_msg)

if __name__ == '__main__':
    rospy.init_node('data_processing_node', anonymous=True)
    sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs
    initialize_filters(sensor_ids)

    # Subscriber to receive raw sensor data
    rospy.Subscriber('sensor_data', String, sensor_data_callback)
    # Subscriber to receive contact state
    rospy.Subscriber('/contact_detected', String, contact_detected_callback)

    # Initialize the publisher for processed data
    publisher = rospy.Publisher('processed_sensor_data', String, queue_size=10)

    # Initialize the publisher for diagnostic messages
    diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    # Use ThreadPoolExecutor for parallel processing
    with ThreadPoolExecutor(max_workers=len(sensor_ids)) as executor:
        for sensor_id in sensor_ids:
            executor.submit(process_sensor_data, sensor_id)

    rospy.spin()
