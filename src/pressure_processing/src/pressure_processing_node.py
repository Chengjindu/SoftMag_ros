#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from scipy.signal import cheby1, lfilter, lfilter_zi
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import time

# Global variables for storing pressure data and filter configurations
pressure_data = {}
pressure_filters = {}
pressure_publishers = {}

# Filter parameters
sampling_frequency = 60  # Assumed typical sampling frequency for pressure data
passband_frequency = 0.5   # Cutoff frequency for low-pass filter
order = 1
passband_ripple = 2    # Ripple in the passband, in dB

def initialize_pressure_filters(pressure_ids):
    """Initialize Chebyshev type I low-pass filters for each pressure ID."""
    global pressure_filters
    nyquist_freq = sampling_frequency / 2
    normalized_passband_freq = passband_frequency / nyquist_freq

    b, a = cheby1(N=order, rp=passband_ripple, Wn=normalized_passband_freq, btype='low')
    zi = lfilter_zi(b, a)

    for pid in pressure_ids:
        pressure_filters[pid] = {'b': b, 'a': a, 'zi': zi * np.zeros(len(b) - 1)}

def pressure_data_callback(msg, pressure_id):
    """Callback function for pressure data topics."""
    data = msg.data
    filtered_data = apply_filter(data, pressure_id)
    publish_filtered_data(pressure_id, filtered_data)

def apply_filter(data, pressure_id):
    """Filter pressure data using predefined filter coefficients."""
    filter_config = pressure_filters[pressure_id]
    filtered_data, zi = lfilter(filter_config['b'], filter_config['a'], [data], zi=filter_config['zi'])
    filter_config['zi'] = zi  # Update the filter state
    return filtered_data[0]

def publish_filtered_data(pressure_id, data):
    """Publish filtered data."""
    if pressure_id in pressure_publishers:
        pressure_publishers[pressure_id].publish(Float32(data=data))

def setup_publishers(pressure_ids):
    """Setup ROS publishers for filtered pressure data."""
    global pressure_publishers
    for pid in pressure_ids:
        topic_name = f'pressure_filtered_{pid}'
        pressure_publishers[pid] = rospy.Publisher(topic_name, Float32, queue_size=10)

def process_pressure_data(pressure_id):
    """Continuously process incoming data for a given pressure sensor."""
    while not rospy.is_shutdown():
        if pressure_id in pressure_data:
            data = pressure_data[pressure_id]
            filtered_data = apply_filter(data, pressure_id)
            publish_filtered_data(pressure_id, filtered_data)
            time.sleep(0.1)  # simulate some processing delay

def main():
    rospy.init_node('pressure_processing_node')
    pressure_ids = ['P1', 'P2']  # Example pressure sensor IDs

    initialize_pressure_filters(pressure_ids)
    setup_publishers(pressure_ids)

    # Subscribe to pressure readings
    rospy.Subscriber('pressure_reading1', Float32, pressure_data_callback, callback_args='P1')
    rospy.Subscriber('pressure_reading2', Float32, pressure_data_callback, callback_args='P2')

    # Use ThreadPoolExecutor to handle multiple pressure data streams concurrently
    with ThreadPoolExecutor(max_workers=len(pressure_ids)) as executor:
        for pid in pressure_ids:
            executor.submit(process_pressure_data, pid)

    rospy.spin()

if __name__ == '__main__':
    main()
