#!/usr/bin/env python3
import rospy
from rospy import Time
from std_msgs.msg import Float32MultiArray, String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from scipy.signal import cheby1, lfilter, lfilter_zi
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import threading
import time


# Global variables for storing sensor data and filter configurations
sensor_data = {}
filters = {}
delay = None
data_stablized = {}
contact_detected = {}
contact_status = False
contact_check_flag = False
recalibration_flag = False
data_count = 0
processing_count = 0
publishing_count = 0
start_time = 0.0
contact_false_count = 0

# Filter parameters
initilization_window = 500
stablized_average_window = 100
sampling_frequency = 50
passband_frequency = 3
order = 2
passband_ripple = 0.8
recalibration_interval = 60  # Time interval to check for recalibration (60 seconds)
contact_threshold = 200  # Number of continuous false readings required to trigger recalibration


def individual_sensor_data_callback(msg, sensor_id): # Callback function for sensor_data topic
    global data_count, start_time

    if start_time == 0.0:      # Initialize start time on first callback
        start_time = time.time()
    data_count += 1  # Increment the data count


    measurements = list(msg.data)   # Directly use the data from Float32MultiArray
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
        rospy.logerr("Error filtering data: %s", e)
        return 0.0, 0.0, 0.0

def measure_data_rate():    # Measure the rate of incoming sensor data
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


def contact_callback(msg):   # Callback for contact detection
    global contact_status, contact_false_count, recalibration_flag, contact_check_flag

    contact_status = msg.data

    if contact_check_flag:
        if not contact_status:
            contact_false_count += 1

            if contact_false_count >= contact_threshold:
                recalibration_flag = True
                contact_check_flag = False  # Stop checking contacts
                contact_false_count = 0
                rospy.loginfo("Recalibration triggered due to continuous no contact")

        else:
            contact_false_count = 0


def zero_sensor_callback(msg):  # Callback to start the zeroing process for sensors
    global recalibration_flag, contact_check_flag

    if msg.data:
        rospy.loginfo("Starting contact check for recalibration")
        contact_check_flag = True  # Enable contact check

        while not recalibration_flag and not rospy.is_shutdown():
            rospy.sleep(2)
        if recalibration_flag:
            recalibrate_sensors()
            recalibration_flag = False
        contact_check_flag = False  # Disable contact check after recalibration

def recalibrate_sensors():  # Recalibrate all sensors
    global sensor_ids

    # Reinitialize filters with the measured sampling frequency
    initialize_filters(sensor_ids, sampling_freq=sampling_frequency)

    rospy.loginfo("Recalibrating sensors")
    for sensor_id in sensor_data.keys():
        zero_sensor(sensor_id)  # Call the zeroing method for each sensor

    recalibration_flag = False
    sensor_zero_publisher.publish(Bool(data=True))  # Publish the completion message

def zero_sensor(sensor_id):     # Function for offset elimination
    global initilization_window, initial_values, data_stablized, sensor_data, filters, delay, contact_status

    while True:
        local_data_stablized = False
        Readout_count = 0
        message_printed = False
        data_points = []

        while Readout_count < initilization_window and not rospy.is_shutdown():
            if contact_status:  # Check for contact during stabilization
                rospy.logwarn(f"Contact detected during stabilization of {sensor_id}. Restarting zeroing process.")
                Readout_count = 0
                data_points = []
                rospy.sleep(1)  # Small delay before retrying
                continue

            data = sensor_data.get(sensor_id)
            if data is not None and len(data) == 3:
                if all(value == 0.0 for value in data):
                    continue

                try:
                    filtered_x, filtered_y, filtered_z = filtering(data, filters[sensor_id])
                    if not message_printed:
                        rospy.loginfo(f"Waiting readout for {sensor_id} to stabilize...")
                        message_printed = True
                    Readout_count += 1

                    if Readout_count >= initilization_window:  # Assuming stabilization
                        rospy.loginfo(f"Data stable for {sensor_id}.")
                        local_data_stablized = True
                        break

                except Exception as e:
                    rospy.logerr(f"Error zeroing data for sensor {sensor_id}: {e}")

        if local_data_stablized:        # Collect stablized data
            rospy.loginfo(f"Collecting data points for averaging for {sensor_id}...")
            for _ in range(stablized_average_window):
                if contact_status:  # Check for contact during averaging
                    rospy.logwarn(f"Contact detected during averaging of {sensor_id}. Restarting zeroing process.")
                    local_data_stablized = False
                    break

                data = sensor_data.get(sensor_id)
                if data is not None and len(data) == 3:
                    filtered_x, filtered_y, filtered_z = filtering(data, filters[sensor_id])
                    data_points.append((filtered_x, filtered_y, filtered_z))
                    rospy.sleep(delay)  # Add a delay between readings to match the data rate

            if not local_data_stablized:
                continue  # Restart the zeroing process if contact was detected


            if data_points:     # Calculate the average of the collected data points
                avg_x = np.mean([point[0] for point in data_points])
                avg_y = np.mean([point[1] for point in data_points])
                avg_z = np.mean([point[2] for point in data_points])

                # Update global variables after local recalibration is done
                initial_values[sensor_id] = (avg_x, avg_y, avg_z)
                rospy.loginfo(f"Averaged initial values for {sensor_id}: {initial_values[sensor_id]}")
                data_stablized[sensor_id] = local_data_stablized
                break  # Exit the loop if zeroing is successful

def process_sensor_data(sensor_id, publisher):      # Process data for a single sensor
    global sensor_data, filters, delay, initial_values, data_stablized, contact_detected, processing_count, publishing_count, start_time

    # if start_time is None:
    #     start_time = time.time()

    # Initialize the contact_detected dictionary with default values for each sensor
    contact_detected[sensor_id] = False

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

                if data_stablized[sensor_id] and initial_values[sensor_id] is not None:  # If data is stabilized, apply deviation elimination
                    filtered_x = filtered_x - initial_values[sensor_id][0]
                    filtered_y = filtered_y - initial_values[sensor_id][1]
                    filtered_z = filtered_z - initial_values[sensor_id][2]

                    all_stabilized = all(data_stablized.values())  # Check if all values in the dictionary are True
                    all_stabilized_publisher.publish(Bool(data = all_stabilized))

                    # Prepare and publish the processed data
                    publish_data = Float32MultiArray()
                    publish_data.data = [filtered_x, filtered_y, filtered_z]
                    publisher.publish(publish_data)

                    # publishing_count += 1
                    # if publishing_count % 100 == 0:
                    #     elapsed_time = time.time() - start_time
                    #     publishing_rate = publishing_count / elapsed_time
                    #     rospy.loginfo(f"Publishing rate for {sensor_id}: {publishing_rate} Hz")

                publish_diagnostics(sensor_id, True, "Data processing successful", filtered_data=(filtered_x, filtered_y, filtered_z))
                rospy.sleep(delay)

            except Exception as e:
                rospy.logerr(f"Error filtering data for sensor {sensor_id}: {e}")
                publish_diagnostics(sensor_id, False, f"Error filtering data for sensor {sensor_id}: {e}")

def publish_diagnostics(sensor_id, status, message, filtered_data=None):
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = Time.now()
    status_msg = DiagnosticStatus()
    status_msg.name = f"DataProcessingNode: {sensor_id}"
    status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.ERROR
    status_msg.message = message
    status_msg.hardware_id = sensor_id
    status_msg.values = [
        KeyValue("Sensor ID", sensor_id),
        KeyValue("Processing Status", "Success" if status else "Failure")
    ]

    if filtered_data:
        status_msg.values.append(KeyValue("Filtered X", str(filtered_data[0])))
        status_msg.values.append(KeyValue("Filtered Y", str(filtered_data[1])))
        status_msg.values.append(KeyValue("Filtered Z", str(filtered_data[2])))

    diag_msg.status.append(status_msg)
    diag_pub.publish(diag_msg)

if __name__ == '__main__':
    rospy.init_node('data_processing_node')
    # rospy.init_node('data_processing_node', anonymous=True)
    sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs

    for sensor_id in sensor_ids:            # Initialize global variables
        sensor_data[sensor_id] = None
    initial_values = {sensor_id: None for sensor_id in sensor_ids}
    data_stablized = {sensor_id: False for sensor_id in sensor_ids}

    for sensor_id in sensor_ids:    # Subscribe to necessary topics
        rospy.Subscriber(f'sensor_data_{sensor_id}', Float32MultiArray,
                         lambda msg, sid=sensor_id: individual_sensor_data_callback(msg, sid))
    rospy.Subscriber('/contact_detect', Bool, contact_callback)
    rospy.Subscriber('/zero_sensor', Bool, zero_sensor_callback)

    # Initialize ROS publishers
    diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
    all_stabilized_publisher = rospy.Publisher('sensors_stabilized', Bool, queue_size=10)
    sensor_zero_publisher = rospy.Publisher('/sensor_zero', Bool, queue_size=10)

    rospy.sleep(8)  # Run the node for a few seconds to gather data rate information
    if start_time == 0.0:
        rospy.logwarn("No data received from sensors.")
        sampling_frequency = 50  # Default value
    else:
        # Measure and log the data rate
        sampling_frequency = measure_data_rate() / 2
        if sampling_frequency <= 0:
            rospy.logwarn("Measured data rate is invalid.")
            sampling_frequency = 50  # Default value
    rospy.loginfo(f"Setting filter sampling frequency to {sampling_frequency} Hz")

    # Initialize filters with the measured sampling frequency
    initialize_filters(sensor_ids, sampling_freq=sampling_frequency)

    publishers = {sensor_id: rospy.Publisher(f'processed_sensor_data_{sensor_id}', Float32MultiArray, queue_size=10) for sensor_id in sensor_ids}

    for sensor_id in sensor_ids:
        zero_sensor(sensor_id)  # Perform initial zeroing

    # Use ThreadPoolExecutor to process data from sensors concurrently
    with ThreadPoolExecutor(max_workers=len(sensor_ids)) as executor:
        for sensor_id in sensor_ids:
            executor.submit(process_sensor_data, sensor_id, publishers[sensor_id])

    rospy.spin()
