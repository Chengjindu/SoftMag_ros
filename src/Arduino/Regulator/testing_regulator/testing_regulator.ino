#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ArduinoJson.h>

Adafruit_MCP4725 dac; // DAC for pressure control
ros::NodeHandle nh;
int monitorPin = A0;

float desiredPressure = 0.0;
float maxPressure = 35.0;
bool dataStabilized = false; // Flag to check if data is stabilized
bool motorStopped = true;    // Flag to check if motor is stopped
bool forceClosureEnabled = false; // Flag to check if force closure is enabled
bool forceClosureTrigger = false; // Added forceClosureTrigger flag

std_msgs::String graspStableMsg;
std_msgs::String releaseFinishMsg;
std_msgs::Float32 pressure_msg;
ros::Publisher graspStablePub("grasp_stable", &graspStableMsg);
ros::Publisher releaseFinishPub("release_finish", &releaseFinishMsg);
ros::Publisher pressurereadingPub("pressure_reading", &pressure_msg);

void motorStopCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);
  motorStopped = doc["motor_stop"];
}

void forceClosureCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);
  forceClosureEnabled = doc["force_closure_enabled"];
  
  if (doc["force_closure"] && !forceClosureTrigger) {
    if (desiredPressure + 1.0 <= maxPressure) {
      desiredPressure += 0.5;
      forceClosureTrigger = true; // Indicate that force closure has been achieved
    } else {
      forceClosureTrigger = true;
    }
    
    StaticJsonDocument<200> stableDoc;
    stableDoc["grasp_stable_flag"] = true;
    char jsonBuffer[256];
    serializeJson(stableDoc, jsonBuffer);
    graspStableMsg.data = jsonBuffer;
    graspStablePub.publish(&graspStableMsg);
  }
}

void stopallCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);
  if (doc["stop_all_flag"]) {
    desiredPressure = 0.0;
    forceClosureTrigger = false;

    StaticJsonDocument<200> stableDoc;
    char jsonBuffer[256];
    stableDoc["grasp_stable_flag"] = false;
    serializeJson(stableDoc, jsonBuffer);
    graspStableMsg.data = jsonBuffer;
    graspStablePub.publish(&graspStableMsg);
  }
}

void pressureCtrlCallback(const std_msgs::Float32& msg) {
  desiredPressure = msg.data;
}

void processedSensorDataCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);
  dataStabilized = doc["stable_flag"];
}

ros::Subscriber<std_msgs::String> motorStopSub("motor_stop", &motorStopCallback);
ros::Subscriber<std_msgs::String> forceClosureSub("force_closure", &forceClosureCallback);
ros::Subscriber<std_msgs::String> stopallSub("stop_all", &stopallCallback);
ros::Subscriber<std_msgs::String> processedSensorDataSub("processed_sensor_data", &processedSensorDataCallback);
ros::Subscriber<std_msgs::Float32> pressureCtrlSub("pressure_ctrl", &pressureCtrlCallback);

void setup() {
  Serial.begin(57600);
  dac.begin(0x60);
  nh.initNode();
  nh.subscribe(motorStopSub);
  nh.subscribe(forceClosureSub);
  nh.subscribe(stopallSub);
  nh.subscribe(processedSensorDataSub);
  nh.subscribe(pressureCtrlSub);
  nh.advertise(pressurereadingPub);
  nh.advertise(graspStablePub);
}

void loop() {
  if (dataStabilized && motorStopped) {
    uint16_t dacValue = (uint16_t)((desiredPressure / 50.0) * 4095);
    dac.setVoltage(dacValue, false);
    
    if (forceClosureEnabled && desiredPressure < maxPressure) {
      desiredPressure += 1.0;
      if (desiredPressure >= maxPressure) {
        desiredPressure = maxPressure;
        forceClosureTrigger = true;
        
        StaticJsonDocument<200> stableDoc;
        stableDoc["grasp_stable_flag"] = true;
        char jsonBuffer[256];
        serializeJson(stableDoc, jsonBuffer);
        graspStableMsg.data = jsonBuffer;
        graspStablePub.publish(&graspStableMsg);
      }

      uint16_t dacValue = (uint16_t)((desiredPressure / 50.0) * 4095);
      dac.setVoltage(dacValue, false);

      delay(100);
    }
  }

  float sensorValue = analogRead(monitorPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float pressure = ((voltage - 1) / 4.0) * 100.0;

  pressure_msg.data = pressure;
  pressurereadingPub.publish(&pressure_msg);
  
  nh.spinOnce();
  delay(100);
}

