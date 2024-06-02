#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ArduinoJson.h>

Adafruit_MCP4725 dac;
ros::NodeHandle nh; 
int monitorPin = A0; 

float desiredPressure = 0.0; 
float maxPressure = 35.0;
bool motorTrigger = false;
bool forceClosureTrigger = false;

std_msgs::String graspStableMsg;
std_msgs::String releaseFinishMsg;
std_msgs::Float32 pressure_msg;
ros::Publisher graspStablePub("grasp_stable", &graspStableMsg);
ros::Publisher releaseFinishPub("release_finish", &releaseFinishMsg);
ros::Publisher pressurereadingPub("pressure_reading", &pressure_msg);

void motorStopTriggerCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);
  bool motor_stop_trigger = doc["motor_stop_trigger"];
  motorTrigger = motor_stop_trigger;
}

void forceClosureCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);

  bool force_closure = doc["force_closure"];
  
  if (force_closure && !forceClosureTrigger && motorTrigger) {
    if (desiredPressure + 1.0 <= maxPressure) {
      desiredPressure += 0.5;
      forceClosureTrigger = true;
    } else {
      forceClosureTrigger = true;
    }
    motorTrigger = false;

    StaticJsonDocument<200> stableDoc;
    stableDoc["grasp_stable_flag"] = true;
    char jsonBuffer[256];
    serializeJson(stableDoc, jsonBuffer);
    graspStableMsg.data = jsonBuffer;
    graspStablePub.publish(&graspStableMsg);
  }
}

void releaseCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);
  bool release_flag = doc["release_flag"];

  if (release_flag) {
    desiredPressure = 0.0;
  }

  StaticJsonDocument<200> stableDoc;
  stableDoc["release_finish_flag"] = true;
  char jsonBuffer[256];
  serializeJson(stableDoc, jsonBuffer);
  releaseFinishMsg.data = jsonBuffer;
  releaseFinishPub.publish(&releaseFinishMsg);

  stableDoc["grasp_stable_flag"] = false;
  serializeJson(stableDoc, jsonBuffer);
  graspStableMsg.data = jsonBuffer;
  graspStablePub.publish(&graspStableMsg);
}

void restartCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);

  bool restart_flag = doc["restart_flag"];
  if (restart_flag) {
    motorTrigger = false;
    forceClosureTrigger = false;
  }
}

void stopallCallback(const std_msgs::String& msg) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, msg.data);

  bool stop_all_flag = doc["stop_all_flag"];
  if (stop_all_flag) {
    desiredPressure = 0.0;
    motorTrigger = false;
    forceClosureTrigger = false;

    StaticJsonDocument<200> stableDoc;
    char jsonBuffer[256];
    stableDoc["grasp_stable_flag"] = false;
    serializeJson(stableDoc, jsonBuffer);
    graspStableMsg.data = jsonBuffer;
    graspStablePub.publish(&graspStableMsg);
  }
}

ros::Subscriber<std_msgs::String> motorStopTriggerSub("motor_stop_trigger", &motorStopTriggerCallback);
ros::Subscriber<std_msgs::String> forceClosureSub("force_closure", &forceClosureCallback);
ros::Subscriber<std_msgs::String> releaseSub("release", &releaseCallback);
ros::Subscriber<std_msgs::String> restartSub("restart", &restartCallback);
ros::Subscriber<std_msgs::String> stopallSub("stop_all", &stopallCallback);

void setup() {
  Serial.begin(57600);
  dac.begin(0x60);
  nh.initNode();
  nh.subscribe(motorStopTriggerSub);
  nh.subscribe(forceClosureSub);
  nh.subscribe(releaseSub);
  nh.subscribe(restartSub);
  nh.subscribe(stopallSub);
  nh.advertise(pressurereadingPub);
  nh.advertise(graspStablePub);
  nh.advertise(releaseFinishPub);
}

void loop() {

  uint16_t dacValue = (uint16_t)((desiredPressure / 50.0) * 4095);
  dac.setVoltage(dacValue, false);

  if (motorTrigger && !forceClosureTrigger && desiredPressure < maxPressure) {
    desiredPressure += 1.0; 
    if (desiredPressure >= maxPressure) {
      desiredPressure = maxPressure;
      motorTrigger = false;
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

  float sensorValue = analogRead(monitorPin); 
  float voltage = sensorValue * (5.0 / 1023.0); 
  float pressure = ((voltage - 1) / 4.0) * 100.0; 

  pressure_msg.data = pressure;
  pressurereadingPub.publish(&pressure_msg);

  nh.spinOnce();
  delay(100);
}

