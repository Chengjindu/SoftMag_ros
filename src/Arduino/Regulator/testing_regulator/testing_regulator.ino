#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ArduinoJson.h>

Adafruit_MCP4725 dac1; // DAC for pressure control
Adafruit_MCP4725 dac2; // DAC for pressure control
int monitorPin1 = A0;
int monitorPin2 = A1;

ros::NodeHandle nh;

float desiredPressure1 = 0.0;
float desiredPressure2 = 0.0;
float maxPressure = 30.0;
float pressureCoefficient = 0.87;

bool dataStabilized = false; // Flag to check if data is stabilized
bool motorStopped = true;    // Flag to check if motor is stopped
bool forceClosureEnabled = false; // Flag to check if force closure is enabled
bool forceClosureTrigger = false; // Added forceClosureTrigger flag

std_msgs::Bool graspStableMsg;
ros::Publisher graspStablePub("grasp_stable", &graspStableMsg);

std_msgs::Float32 pressure_msg1;
ros::Publisher pressurereadingPub1("pressure_reading1", &pressure_msg1);
std_msgs::Float32 pressure_msg2;
ros::Publisher pressurereadingPub2("pressure_reading2", &pressure_msg2);

void motorStopCallback(const std_msgs::Bool& msg) {
  motorStopped = msg.data;
}

void forceClosureCallback(const std_msgs::Bool& msg) {
  bool force_closure = msg.data;
  
  if (forceClosureEnabled && force_closure && !forceClosureTrigger && motorStopped) {
    desiredPressure1 += 0.5;
    desiredPressure2 += 0.5;
    if (desiredPressure1 + 1.0 > pressureCoefficient * maxPressure) {
      desiredPressure1 = pressureCoefficient * maxPressure;
    }
    if (desiredPressure2 + 1.0 > maxPressure) {
      desiredPressure2 = maxPressure;
    }
    forceClosureTrigger = true;

    graspStableMsg.data = true;
    graspStablePub.publish(&graspStableMsg);
  }
}

void stopallCallback(const std_msgs::Bool& msg) {
  bool stop_all_flag = msg.data;

  if (stop_all_flag) {
    desiredPressure1 = 0.0;
    desiredPressure2 = 0.0;
    forceClosureTrigger = false;

    graspStableMsg.data = false;
    graspStablePub.publish(&graspStableMsg);
  }
}

void sensorsStabilizedCallback(const std_msgs::Bool& msg) {
  dataStabilized = msg.data;
}

void maxPressureCallback(const std_msgs::Float32& msg) {
  maxPressure = msg.data;
}

void pressureCoefficientCallback(const std_msgs::Float32& msg) {
  pressureCoefficient = msg.data;
}

void pressureCtrlCallback(const std_msgs::Float32& msg) {
  desiredPressure1 = pressureCoefficient * msg.data;
  desiredPressure2 = msg.data;
}

ros::Subscriber<std_msgs::Bool> motorStopSub("motor_stop", &motorStopCallback);
ros::Subscriber<std_msgs::Bool> forceClosureSub("force_closure", &forceClosureCallback);
ros::Subscriber<std_msgs::Bool> stopallSub("stop_all", &stopallCallback);
ros::Subscriber<std_msgs::Bool> sensorsStabilizedSub("sensors_stabilized", &sensorsStabilizedCallback);
ros::Subscriber<std_msgs::Float32> maxPressureSub("max_pressure", &maxPressureCallback);
ros::Subscriber<std_msgs::Float32> pressureCtrlSub("pressure_ctrl", &pressureCtrlCallback);
ros::Subscriber<std_msgs::Float32> pressureCoefficientSub("pressure_coefficient", &pressureCoefficientCallback);


void setup() {
  Serial.begin(57600);
  dac1.begin(0x60);
  dac2.begin(0x61);
  nh.initNode();
  nh.subscribe(motorStopSub);
  nh.subscribe(forceClosureSub);
  nh.subscribe(stopallSub);
  nh.subscribe(sensorsStabilizedSub);
  nh.subscribe(maxPressureSub);
  nh.subscribe(pressureCtrlSub);
  nh.subscribe(pressureCoefficientSub);
  nh.advertise(pressurereadingPub1);
  nh.advertise(pressurereadingPub2);
  nh.advertise(graspStablePub);
}

void loop() {

  if (dataStabilized && motorStopped) {
    if ((!forceClosureEnabled && desiredPressure2 < maxPressure) || 
    (forceClosureEnabled && !forceClosureTrigger && desiredPressure2 < maxPressure)) {
      if (desiredPressure2 >= maxPressure) {
        desiredPressure1 = pressureCoefficient * maxPressure;
        desiredPressure2 = maxPressure;
        forceClosureTrigger = true;
        
        graspStableMsg.data = true;
      graspStablePub.publish(&graspStableMsg);
      }
    }
  }

  uint16_t dacValue1 = (uint16_t)((desiredPressure1 / 50) * 4095);
  uint16_t dacValue2 = (uint16_t)((desiredPressure2 / 50) * 4095);
  dac1.setVoltage(dacValue1, false);
  dac2.setVoltage(dacValue2, false);

  float sensorValue1 = analogRead(monitorPin1);
  float sensorValue2 = analogRead(monitorPin2);
  float voltage1 = sensorValue1 * (5.0 / 1023.0);
  float voltage2 = sensorValue2 * (5.0 / 1023.0);
  float pressure1 = ((voltage1 - 1) / 4.0) * 100.0;
  float pressure2 = ((voltage2 - 1) / 4.0) * 100.0;

  pressure_msg1.data = pressure1;
  pressure_msg2.data = pressure2;
  pressurereadingPub1.publish(&pressure_msg1);
  pressurereadingPub2.publish(&pressure_msg2);
  
  nh.spinOnce();
  delay(15);
}

