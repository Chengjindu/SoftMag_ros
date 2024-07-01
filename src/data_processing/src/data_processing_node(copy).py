#!/usr/bin/env python3
import rospy
from rospy import Time
from std_msgs.msg import Float32MultiArray, String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import json
from scipy.signal import cheby1, lfilter, lfilter_zi
from concurrent.futures import ThreadPoolExecutor
import time


# Global variables for storing sensor data and filter configurations
sensor_data = {}
filters = {}
delay = None
data_stablized = {}
contact_detected = {}
data_count = 0
processing_count = 0
publishing_count = 0
start_time = 0.0

# Filter parameters
sampling_frequency = 50
passband_frequency = 3
order = 2
passband_ripple = 0.8

def individual_sensor_data_callback(msg, sensor_id): # Callback function for sensor_data topic
    global data_count, start_time

    if start_time == 0.0:      # Initialize start time on first callback
        start_time = time.time()

    data_count += 1  # Increment the data count

    # Directly use the data from Float32MultiArray
    measurements = list(msg.data)
    sensor_data[sensor_id] = measurements

def initialize_filters(sensor_ids, sampling_freq=sampling_frequency, passband_freq=passband_frequency, order=order, passband_ripple=passband_ripple):
    global filters, delay
    # passband_freq specifies the cutoff frequency for the filter
    # Frequencies below this value are allowed to pass, while those above are attenuated
    # order defines the filter's complexity and steepness of the roll-off
    # Higher order filters have a steeper drop-off but can introduce more delay and potential ringing artifacts
    # passband_ripple specifies the allowed ripple in the passband (in dB)
    # It defines how much the filter's response can deviate from flatness within the passband
    # Lower values result in a flatter response, but achieving this may require a higher order filter

    min_sampling_freq = 10.0        # Set a minimum sampling frequency threshold
    if sampling_freq < min_sampling_freq:
        rospy.logwarn(
            f"Sampling frequency {sampling_freq} Hz is too low. Setting to minimum threshold of {min_sampling_freq} Hz.")
        sampling_freq = min_sampling_freq

    nyquist_freq = sampling_freq / 2    # The highest frequency that can be effectively captured by the filter without encountering aliasing.
    normalized_passband_freq = passband_freq / nyquist_freq     # Normalize the passband frequency with respect to the Nyquist frequency.
    # Normalization is necessary for the digital filter design, as it works with frequencies in a unit circle representation.

    if not (0 < normalized_passband_freq < 1):
        raise ValueError(
            "Normalized passband frequency is out of range. Please check the sampling frequency and passband frequency.")

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

def filtering(data, filter_config):     # Function to apply filter to the data
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

def measure_data_rate():
    global data_count, start_time
    rate = 0.0

    if start_time == 0.0:
        rospy.logwarn("Start time is not set. Data rate cannot be measured.")
        return rate

    elapsed_time = time.time() - start_time     # Calculate the elapsed time

    if elapsed_time > 0:
        rate = data_count / elapsed_time        # Calculate the data rate
        rospy.loginfo(f"Incoming data rate: {rate} Hz")

    return rate

# Process data for a single sensor
def process_sensor_data(sensor_id, publisher):
    global sensor_data, filters, delay, data_stablized, contact_detected, processing_count, publishing_count, start_time

    # if start_time is None:
    #     start_time = time.time()

    # Initialize the contact_detected dictionary with default values for each sensor
    contact_detected[sensor_id] = False

    # Initialize state tracking dictionaries outside the loop
    Readout_count = 0
    initial_values = None
    data_stablized[sensor_id] = False
    message_printed = False

    while not rospy.is_shutdown():
        data = sensor_data.get(sensor_id)

        if data is not None and len(data) == 3:
            if all(value == 0.0 for value in data):
                rospy.logwarn(f"Received zero data for sensor {sensor_id}. Skipping this data point.")
                continue

            try:
                filtered_x, filtered_y, filtered_z = filtering(data, filters[sensor_id])

                # processing_count += 1
                # if processing_count % 100 == 0:
                #     elapsed_time = time.time() - start_time
                #     processing_rate = processing_count / elapsed_time
                #     rospy.loginfo(f"Processing rate for {sensor_id}: {processing_rate} Hz")

                if not data_stablized[sensor_id] and initial_values is None:
                    if not message_printed:
                        rospy.loginfo(f"Waiting readout for {sensor_id} to stabilize...")
                        message_printed = True
                    Readout_count += 1

                    if Readout_count >= 100:  # Assuming stabilization after 100 readings
                        rospy.loginfo(f"data stable for {sensor_id}.")
                        initial_values = (filtered_x, filtered_y, filtered_z)  # Set initial values for deviation elimination
                        data_stablized[sensor_id] = True

                if data_stablized[sensor_id] and initial_values is not None:  # If data is stabilized, apply deviation elimination
                    filtered_x = filtered_x - initial_values[0]
                    filtered_y = filtered_y - initial_values[1]
                    filtered_z = filtered_z - initial_values[2]

                    all_stabilized = all(data_stablized.values())  # Check if all values in the dictionary are True
                    all_stabilized_publisher.publish(Bool(data=all_stabilized))

                    # Prepare and publish the processed data
                    publish_data = Float32MultiArray()
                    publish_data.data = [filtered_x, filtered_y, filtered_z]
                    publisher.publish(publish_data)

                    # publishing_count += 1
                    # if publishing_count % 100 == 0:
                    #     elapsed_time = time.time() - start_time
                    #     publishing_rate = publishing_count / elapsed_time
                    #     rospy.loginfo(f"Publishing rate for {sensor_id}: {publishing_rate} Hz")

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

    if filtered_data:  # If filtered data is provided, add it to the diagnostics message
        status_msg.values.append(KeyValue("Filtered X", str(filtered_data[0])))
        status_msg.values.append(KeyValue("Filtered Y", str(filtered_data[1])))
        status_msg.values.append(KeyValue("Filtered Z", str(filtered_data[2])))

    diag_msg.status.append(status_msg)
    diag_pub.publish(diag_msg)

if __name__ == '__main__':
    rospy.init_node('data_processing_node', anonymous=True)
    sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs

    # Initialize sensor data dictionary
    for sensor_id in sensor_ids:
        sensor_data[sensor_id] = None

    # Subscribe to sensor data topics
    for sensor_id in sensor_ids:
        rospy.Subscriber(f'sensor_data_{sensor_id}', Float32MultiArray,
                         lambda msg, sid=sensor_id: individual_sensor_data_callback(msg, sid))

    # Initialize the publisher for diagnostic messages
    diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    all_stabilized_publisher = rospy.Publisher('sensors_stabilized', Bool, queue_size=10)

    rospy.sleep(5)  # Run the node for a few seconds to gather data rate information

    if start_time == 0.0:
        rospy.logwarn("No data received from sensors. Using default sampling frequency of 50 Hz.")
        sampling_frequency = 50  # Default value
    else:
        # Measure and log the data rate
        sampling_frequency = measure_data_rate() / 2
        if sampling_frequency <= 0:
            rospy.logwarn("Measured data rate is invalid. Using default sampling frequency of 50 Hz.")
            sampling_frequency = 50  # Default value

    rospy.loginfo(f"Setting filter sampling frequency to {sampling_frequency} Hz")

    # Initialize filters with the measured sampling frequency
    initialize_filters(sensor_ids, sampling_freq=sampling_frequency)

    publishers = {sensor_id: rospy.Publisher(f'processed_sensor_data_{sensor_id}', Float32MultiArray, queue_size=10) for sensor_id in sensor_ids}


    with ThreadPoolExecutor(max_workers=len(sensor_ids)) as executor:
        for sensor_id in sensor_ids:
            executor.submit(process_sensor_data, sensor_id, publishers[sensor_id])

    rospy.spin()