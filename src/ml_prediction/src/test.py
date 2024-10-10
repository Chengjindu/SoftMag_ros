import os
import pickle
import joblib

def load_model(model_name):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    model_path = os.path.join(dir_path, 'Models', model_name)
    with open(model_path, 'rb') as file:
        model = joblib.load(file)
    return model

def main():
    try:
        actuation_sensor_model_S1 = load_model('actuation_sensor_model_S1.pkl')
        actuation_sensor_model_S2 = load_model('actuation_sensor_model_S2.pkl')
        print("Models loaded successfully.")
        print("Actuation Sensor Model S1:", actuation_sensor_model_S1)
        print("Actuation Sensor Model S2:", actuation_sensor_model_S2)
    except Exception as e:
        print("Failed to load models.")
        print("Error:", str(e))

if __name__ == "__main__":
    main()
