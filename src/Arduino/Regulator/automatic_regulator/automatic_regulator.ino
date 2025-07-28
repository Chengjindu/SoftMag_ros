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
float initialPressure =17.5;
float adaptivePressureDelta = 0.0;
float adaptiveTargetPressure = 0.0;
float maxPressure = 25.0;
float pressureCoefficient = 1.10;
float defaultPressureStep = 1.0;
float adaptivelPressureStep = 0.2;

bool contactTrigger = false;
bool forceClosureTrigger = false;

bool adaptive_flag = false;
bool newAdaptiveDeltaReceived = false;

std_msgs::Bool graspStableMsg;
ros::Publisher graspStablePub("grasp_stable", &graspStableMsg);

std_msgs::Bool releaseFinishMsg;
ros::Publisher releaseFinishPub("release_finish", &releaseFinishMsg);

std_msgs::Float32 pressure_msg1;
ros::Publisher pressurereadingPub1("pressure_reading1", &pressure_msg1);

std_msgs::Float32 pressure_msg2;
ros::Publisher pressurereadingPub2("pressure_reading2", &pressure_msg2);

// std_msgs::Bool contactTriggerMsg;
// ros::Publisher contactTriggerPub("debug/contact_trigger", &contactTriggerMsg);

// std_msgs::Bool forceClosureTriggerMsg;
// ros::Publisher forceClosureTriggerPub("debug/force_closure_trigger", &forceClosureTriggerMsg);

// std_msgs::Bool contactTriggerEchoMsg;
// ros::Publisher contactTriggerEchoPub("debug/contact_trigger_echo", &contactTriggerEchoMsg);



void contactTriggerCallback(const std_msgs::Bool& msg) {
  contactTrigger = msg.data;

  // contactTriggerEchoMsg.data = msg.data;
  // contactTriggerEchoPub.publish(&contactTriggerEchoMsg);
}

void forceClosureCallback(const std_msgs::Bool& msg) {
  bool force_closure = msg.data;
  
  if (force_closure && !forceClosureTrigger && contactTrigger) {
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

void adaptiveFlagCallback(const std_msgs::Bool& msg) {
  adaptive_flag = msg.data;
}

void adaptivePressureDeltaCallback(const std_msgs::Float32& msg) {
  if (!adaptive_flag) return;  // Only respond when adaptive mode is active

  adaptivePressureDelta = msg.data;
  adaptiveTargetPressure = desiredPressure2 + adaptivePressureDelta;

  if (adaptiveTargetPressure > maxPressure) {
    adaptiveTargetPressure = maxPressure;
  }

  newAdaptiveDeltaReceived = true;
}


void releaseCallback(const std_msgs::Bool& msg) {
  bool release_flag = msg.data;

  if (release_flag) {
    desiredPressure1 = 0.0;
    desiredPressure2 = 0.0;
    contactTrigger = false;
    forceClosureTrigger = false;
    releaseFinishMsg.data = true;
    releaseFinishPub.publish(&releaseFinishMsg);
    nh.loginfo("Published: release finished.");
  }

  graspStableMsg.data = false;
  graspStablePub.publish(&graspStableMsg);
}

void restartCallback(const std_msgs::Bool& msg) {
  bool restart_flag = msg.data;
  if (restart_flag) {
    contactTrigger = false;
    forceClosureTrigger = false;
  }
}

void stopallCallback(const std_msgs::Bool& msg) {
  bool stop_all_flag = msg.data;
  if (stop_all_flag) {
    desiredPressure1 = 0.0;
    desiredPressure2 = 0.0;
    contactTrigger = false;
    forceClosureTrigger = false;

    graspStableMsg.data = false;
    graspStablePub.publish(&graspStableMsg);
  }
}

void maxPressureCallback(const std_msgs::Float32& msg) {
  maxPressure = msg.data;
}

void pressureCoefficientCallback(const std_msgs::Float32& msg) {
  pressureCoefficient = msg.data;
}

ros::Subscriber<std_msgs::Bool> contactTriggerSub("contact_trigger", &contactTriggerCallback);
ros::Subscriber<std_msgs::Bool> forceClosureSub("force_closure", &forceClosureCallback);
ros::Subscriber<std_msgs::Bool> releaseSub("release", &releaseCallback);
ros::Subscriber<std_msgs::Bool> restartSub("restart", &restartCallback);
ros::Subscriber<std_msgs::Bool> stopallSub("stop_all", &stopallCallback);
ros::Subscriber<std_msgs::Float32> maxPressureSub("max_pressure", &maxPressureCallback);
ros::Subscriber<std_msgs::Float32> pressureCoefficientSub("pressure_coefficient", &pressureCoefficientCallback);

ros::Subscriber<std_msgs::Bool> adaptiveFlagSub("adaptive_flag", &adaptiveFlagCallback);
ros::Subscriber<std_msgs::Float32> adaptivePressureDeltaSub("adaptive_pressure_delta", &adaptivePressureDeltaCallback);


void setup() {
  Serial.begin(57600);
  dac1.begin(0x60);
  dac2.begin(0x61);
  nh.initNode();
  nh.subscribe(contactTriggerSub);
  nh.subscribe(forceClosureSub);
  nh.subscribe(adaptiveFlagSub);
  nh.subscribe(adaptivePressureDeltaSub);
  nh.subscribe(releaseSub);
  nh.subscribe(restartSub);
  nh.subscribe(stopallSub);
  nh.subscribe(maxPressureSub);
  nh.subscribe(pressureCoefficientSub);
  nh.advertise(pressurereadingPub1);
  nh.advertise(pressurereadingPub2);
  nh.advertise(graspStablePub);
  nh.advertise(releaseFinishPub);
  // nh.advertise(contactTriggerPub);
  // nh.advertise(forceClosureTriggerPub);
  // nh.advertise(contactTriggerEchoPub);
}

void loop() {
  // --- Default pressurization phase: until initial grasp pressure is reached ---
  if (contactTrigger && !forceClosureTrigger && desiredPressure2 < initialPressure) {
    desiredPressure1 += pressureCoefficient * defaultPressureStep;
    desiredPressure2 += defaultPressureStep;

    if (desiredPressure2 >= initialPressure) {
      desiredPressure2 = initialPressure;
      desiredPressure1 = pressureCoefficient * initialPressure;
      forceClosureTrigger = true;

      graspStableMsg.data = true;
      graspStablePub.publish(&graspStableMsg);
    }

  }

  // --- Adaptive phase: stepwise control with delta ---
  if (adaptive_flag && forceClosureTrigger && newAdaptiveDeltaReceived) {
    if (desiredPressure2 < adaptiveTargetPressure) {
      desiredPressure2 += adaptivelPressureStep;
      desiredPressure1 += pressureCoefficient * adaptivelPressureStep;

      if (desiredPressure2 >= adaptiveTargetPressure) {
        desiredPressure2 = adaptiveTargetPressure;
        desiredPressure1 = pressureCoefficient * adaptiveTargetPressure;
        newAdaptiveDeltaReceived = false;  // Delta complete
      }

      if (desiredPressure1 > maxPressure || desiredPressure2 > maxPressure) {
        desiredPressure1 = pressureCoefficient * maxPressure;
        desiredPressure2 = maxPressure;
      }

    } else {
      newAdaptiveDeltaReceived = false;  // Already reached target
    }

    if (desiredPressure1 > maxPressure || desiredPressure2 > maxPressure) {
      desiredPressure1 = pressureCoefficient * maxPressure;
      desiredPressure2 = maxPressure;
    }

  }


  // --- DAC Control ---
  uint16_t dacValue1 = (uint16_t)((desiredPressure1 / 50) * 4095);
  uint16_t dacValue2 = (uint16_t)((desiredPressure2 / 50) * 4095);
  dac1.setVoltage(dacValue1, false);
  dac2.setVoltage(dacValue2, false);

  // --- Pressure sensing and publishing ---
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
  delay(100);
}


