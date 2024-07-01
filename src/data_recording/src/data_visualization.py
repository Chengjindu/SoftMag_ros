import json
import matplotlib.pyplot as plt
import os
import glob

def read_data(file_path):
    with open(file_path, 'r') as file:
        data = [json.loads(line) for line in file]
    return data

def extract_data(data):
    timestamps = [entry['timestamp'] for entry in data]
    start_time = timestamps[0]
    times = [t - start_time for t in timestamps]

    sensor_data_S1 = {
        'x': [entry['sensor_data_S1_x'] for entry in data],
        'y': [entry['sensor_data_S1_y'] for entry in data],
        'z': [entry['sensor_data_S1_z'] for entry in data]
    }

    sensor_data_S2 = {
        'x': [entry['sensor_data_S2_x'] for entry in data],
        'y': [entry['sensor_data_S2_y'] for entry in data],
        'z': [entry['sensor_data_S2_z'] for entry in data]
    }

    prediction_positions = {
        'S1': [entry['prediction_position_S1'] for entry in data],
        'S2': [entry['prediction_position_S2'] for entry in data]
    }

    prediction_forces = {
        'S1': [entry['prediction_force_S1'] for entry in data],
        'S2': [entry['prediction_force_S2'] for entry in data]
    }

    pressures = {
        'pressure1': [entry['pressure_reading1'] for entry in data],
        'pressure2': [entry['pressure_reading2'] for entry in data]
    }

    return times, sensor_data_S1, sensor_data_S2, prediction_positions, prediction_forces, pressures

def plot_data(times, sensor_data_S1, sensor_data_S2, prediction_positions, prediction_forces, pressures):
    plt.figure(figsize=(12, 8))

    # Plot sensor data for S1
    plt.subplot(4, 1, 1)
    plt.plot(times, sensor_data_S1['x'], label='S1_x')
    plt.plot(times, sensor_data_S1['y'], label='S1_y')
    plt.plot(times, sensor_data_S1['z'], label='S1_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Sensor S1')
    plt.legend()
    plt.title('Sensor Data S1')

    # Plot sensor data for S2
    plt.subplot(4, 1, 2)
    plt.plot(times, sensor_data_S2['x'], label='S2_x')
    plt.plot(times, sensor_data_S2['y'], label='S2_y')
    plt.plot(times, sensor_data_S2['z'], label='S2_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Sensor S2')
    plt.legend()
    plt.title('Sensor Data S2')

    # Plot prediction forces for S1 and S2
    plt.subplot(4, 1, 3)
    plt.plot(times, prediction_forces['S1'], label='Force S1')
    plt.plot(times, prediction_forces['S2'], label='Force S2')
    plt.xlabel('Time (s)')
    plt.ylabel('Forces')
    plt.legend()
    plt.title('Prediction Forces')

    # Plot pressures
    plt.subplot(4, 1, 4)
    plt.plot(times, pressures['pressure1'], label='Pressure 1')
    plt.plot(times, pressures['pressure2'], label='Pressure 2')
    plt.xlabel('Time (s)')
    plt.ylabel('Pressures')
    plt.legend()
    plt.title('Pressures')

    plt.tight_layout()
    plt.show()

    # Plot prediction positions for S1 and S2
    plt.figure()
    plt.plot(times, prediction_positions['S1'], label='Position S1')
    plt.plot(times, prediction_positions['S2'], label='Position S2')
    plt.xlabel('Time (s)')
    plt.ylabel('Positions')
    plt.legend()
    plt.title('Prediction Positions')
    plt.show()

def find_latest_file(directory, extension='*.txt'):
    list_of_files = glob.glob(os.path.join(directory, extension))
    latest_file = max(list_of_files, key=os.path.getmtime)
    return latest_file

def main():
    directory = '.'  # Current directory
    file_path = find_latest_file(directory)
    data = read_data(file_path)
    times, sensor_data_S1, sensor_data_S2, prediction_positions, prediction_forces, pressures = extract_data(data)
    plot_data(times, sensor_data_S1, sensor_data_S2, prediction_positions, prediction_forces, pressures)

if __name__ == '__main__':
    main()
