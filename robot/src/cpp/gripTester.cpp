

#include <Arduino.h>
#include <iostream>
#include <cmath> 

using namespace std;

double pi = 3.14159265358979323846;
double deg_to_rad = pi / 180;

const int PWM_CHANNEL_BICEP = 0;
const int PWM_CHANNEL_FOREARM = 1;
const int PWM_CHANNEL_SHOULDER = 2;
const int PWM_CHANNEL_HAND = 3;
const int PWM_CHANNEL_GRIPPER = 4;

const int FREQ = 50;
const int PWM_RESOLUTION = 12;

const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION)) - 1;
const int GPIO_PIN = 18;


//bicep values

bool newDutyBicep = false;
int potValueBicep;
float duty_value_bicep;
int dutyBicep;
int prevDutyBicep;
double bicepLength;

//forearm values

bool newDutyForearm = false;
int potValueForearm;
float duty_value_forearm;
int dutyForearm;
int prevDutyForearm;
double forearmLength;

//shoulder values

bool newDutyShoulder = false;
int potValueShoulder;
float duty_value_shoulder;
int dutyShoulder;
int prevDutyShoulder;


//hand values

bool newDutyHand = false;
int potValueHand;
float duty_value_hand;
int dutyHand;
int prevDutyHand;


//gripper values

bool newDutyGripper = false;
int potValueGripper;
float duty_value_gripper;
int dutyGripper;
int prevDutyGripper;

//x,y calculation variables
int length = 0;

bool gripTF = true;

// put function declarations here:

void setup() {
  Serial.begin(115200);

  ledcSetup(PWM_CHANNEL_GRIPPER, FREQ, PWM_RESOLUTION);
  ledcAttachPin(21, PWM_CHANNEL_GRIPPER); 
}
bool isCurrentlyGripping = false; 

void grip(){
    for (int duty_cycle = 180; duty_cycle <= 380 ; duty_cycle++){
      ledcWrite(PWM_CHANNEL_GRIPPER, duty_cycle);
      //delay(0);
    }
}

void ungrip(){
    for (int duty_cycle = 380; duty_cycle >= 180 ; duty_cycle--){
      ledcWrite(PWM_CHANNEL_GRIPPER, duty_cycle);
      //delay(5); 
    }
}

void loop(){
  
    if (Serial.available() > 0) {
        char incomingKey = Serial.read(); // Read the keystroke
        
        if (incomingKey == 'g'){
            // Only sweep if it isn't ALREADY gripping
            if (!isCurrentlyGripping) {
                grip();
                isCurrentlyGripping = true;
            }
        }
        else if (incomingKey == 'u'){
            // Only sweep if it is currently gripping
            if (isCurrentlyGripping) {
                ungrip();
                isCurrentlyGripping = false;
            }
        }
        else if (incomingKey == 'p'){
            printf("\n");
            delay(2000);
        }
    }
    

}
