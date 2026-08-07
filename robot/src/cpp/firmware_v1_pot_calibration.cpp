

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



void setup() {
  Serial.begin(115200);
  bicepLength = 14.5;
  ledcSetup(PWM_CHANNEL_BICEP, FREQ, PWM_RESOLUTION);
  ledcAttachPin(18, PWM_CHANNEL_BICEP);

  potValueBicep = analogRead(26);
  //duty_value_bicep = (((float)potValueBicep/4095)*80 + 400);
  duty_value_bicep = (((float)potValueBicep/4095)*192 + 288);
  dutyBicep = (int)round(duty_value_bicep);
  prevDutyBicep = dutyBicep;



  ledcSetup(PWM_CHANNEL_FOREARM, FREQ, PWM_RESOLUTION);
  ledcAttachPin(19, PWM_CHANNEL_FOREARM);

  potValueForearm = analogRead(25);
  //duty_value_forearm = ((((float)potValueForearm/4095)*270) + 130);
  duty_value_forearm = ((((float)potValueForearm/4095)*200) + 105);
  dutyForearm = (int)round(duty_value_forearm);
  prevDutyForearm = prevDutyForearm;
  forearmLength = 15.5;
  


  ledcSetup(PWM_CHANNEL_SHOULDER, FREQ, PWM_RESOLUTION);
  ledcAttachPin(5, PWM_CHANNEL_SHOULDER); 

  
  potValueShoulder = analogRead(27);
  //duty_value_forearm = ((((float)potValueForearm/4095)*270) + 130);
  duty_value_shoulder = ((((float)potValueShoulder/4095)*400) + 105);
  dutyShoulder= (int)round(duty_value_shoulder);
  prevDutyShoulder = prevDutyShoulder;
  
  ledcSetup(PWM_CHANNEL_HAND, FREQ, PWM_RESOLUTION);
  ledcAttachPin(4, PWM_CHANNEL_HAND); 


  ledcSetup(PWM_CHANNEL_GRIPPER, FREQ, PWM_RESOLUTION);
  ledcAttachPin(21, PWM_CHANNEL_GRIPPER); 

  
  /*
  for (int duty_cycle = 0; duty_cycle <= 305 ; duty_cycle++){
    ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
    delay(1);
  }
  */


  //120
/*
  for (int duty_cycle = 0; duty_cycle <= 300 ; duty_cycle++){
    ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
    delay(.65);
  }
*/
  //delay(100);




}

std::pair<double, double> coordinateCalculator(){
  std::pair<double, double> coordinate;
  int angle = 0;
  double x = 0;
  double y = 0;
  if (dutyShoulder<316){
    angle = 90.0 * (((dutyShoulder-105.0) / (316.0-105.0)));
    x = length*sin(angle*deg_to_rad);
    y = length*cos(angle*deg_to_rad);
  }
  else{
    angle = (180.0 * (((dutyShoulder-105.0) / (520.0-105.0)))) - 90.0;
    y = length*sin(angle*deg_to_rad);
    x = -length*cos(angle*deg_to_rad);
  }
  coordinate.first = x;
  coordinate.second = y;
  return coordinate;
}


double calculateBicepLength(){


  if (dutyBicep >= 338){
    bicepLength = 10.0 * cos(((double)(90.0 -(90.0*(double)(((dutyBicep - 338.0) / (469.0 - 338.0))))))*deg_to_rad);
  }
  else if (dutyBicep < 338){
    bicepLength = -10.0 * cos(((double)((dutyBicep/338.0)) * 90.0)*deg_to_rad);
  }
  return bicepLength;
}

double calculateForearmLength(){

  if (dutyForearm >= 223){
    forearmLength = 16.5 * cos((double)((double)((double)(dutyForearm - 223.0)/111.0)*-30.0)*deg_to_rad);
  }
  else{
    forearmLength = 16.5 * cos((double)((double)((double)(223.0-dutyForearm)/118.0)*45.0)*deg_to_rad);
  }
  return forearmLength;

}

void grip(){
    for (int duty_cycle = 0; duty_cycle <= 120 ; duty_cycle++){
      ledcWrite(PWM_CHANNEL_GRIPPER, duty_cycle);
      delay(1);
    }
}

void ungrip(){
    for (int duty_cycle = 0; duty_cycle <= 335 ; duty_cycle++){
      ledcWrite(PWM_CHANNEL_GRIPPER, duty_cycle);
      delay(1);
    }
}

void loop(){
  
    //ledcWrite(PWM_CHANNEL, 30);        // set the Duty cycle to 50 out of 255
    //delay(15);                       // Wait for 15 mS

    length = calculateForearmLength() + calculateBicepLength();

    if (Serial.available() > 0) {
        char incomingKey = Serial.read(); // Read the keystroke
        if (incomingKey == 'g'){
          grip();
        }
        else if (incomingKey == 'u'){
          ungrip();
        }
        else if (incomingKey == '1'){
          for (int duty_cycle = 0; duty_cycle <= 120 ; duty_cycle++){
            ledcWrite(PWM_CHANNEL_HAND, duty_cycle);
            delay(1);
          }
        }
        else if (incomingKey == '2'){
          for (int duty_cycle = 0; duty_cycle <= 335 ; duty_cycle++){
            ledcWrite(PWM_CHANNEL_HAND, duty_cycle);
            delay(1);
          }
        }
        else if (incomingKey == '3'){
          for (int duty_cycle = 0; duty_cycle <= 460 ; duty_cycle++){
            ledcWrite(PWM_CHANNEL_HAND, duty_cycle);
            delay(1);
          }
        }
        else if (incomingKey == 'p'){
            printf("\n");
            printf("\n");
            printf("\n");
            printf("Bicep duty \n");
            printf("%u", dutyBicep);
            printf("\n");
            cout << calculateBicepLength() << endl;
            //printf("%u", unsigned(calculateBicepLength()));
            printf("\n");

            printf("Forearm duty \n");
            printf("%u", dutyForearm);
            printf("\n");
            cout << calculateForearmLength() << endl;
            printf("\n");
            cout << calculateForearmLength() + calculateBicepLength() << endl;
            printf("\n");
            cout << coordinateCalculator().first << " and " << coordinateCalculator().second;
            printf("\n");
            cout << dutyShoulder << endl;
            printf("\n");
            delay(2000);
        }
        
    }


    
    if (prevDutyBicep!=dutyBicep){
      newDutyBicep = true;
    }    

    if (newDutyBicep){
      if (prevDutyBicep < dutyBicep){      
        for (int duty_cycle = prevDutyBicep; duty_cycle <= dutyBicep ; duty_cycle++){
          ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
          //delay(1);
        }
      }
      else{
        for (int duty_cycle = prevDutyBicep; duty_cycle >= dutyBicep ; duty_cycle--){
          ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
          //delay(1);
        }
      }
      prevDutyBicep = dutyBicep;
      newDutyBicep = false;
    }

    potValueBicep = analogRead(26);
    duty_value_bicep = (((float)potValueBicep/4095)*192 + 288);
    dutyBicep = (int)round(duty_value_bicep);



    if (prevDutyForearm!=dutyForearm){
      newDutyForearm = true;
    }
    //Serial.println(dutyForearm);

    if (newDutyForearm){
      if (prevDutyForearm < dutyForearm){
        for (int duty_cycle = prevDutyForearm; duty_cycle <= dutyForearm ; duty_cycle++){
          ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
          delay(1);
        }
      }
      else{
      for (int duty_cycle = prevDutyForearm; duty_cycle >= dutyForearm ; duty_cycle--){
          ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
          delay(1);
        }
      }
      newDutyForearm = false;
      prevDutyForearm = dutyForearm;

    }

    potValueForearm = analogRead(25);
    duty_value_forearm = ((((float)potValueForearm/4095)*200) + 105);
    dutyForearm = (int)round(duty_value_forearm);

    
    if (prevDutyShoulder!=dutyShoulder){
      newDutyShoulder = true;
    }
    //Serial.println(dutyShoulder);

    if (newDutyShoulder){
      if (prevDutyShoulder < dutyShoulder){
        for (int duty_cycle = prevDutyShoulder; duty_cycle <= dutyShoulder ; duty_cycle++){
          ledcWrite(PWM_CHANNEL_SHOULDER, duty_cycle);
          //delay(1);
        }
      }
      else{
      for (int duty_cycle = prevDutyShoulder; duty_cycle >= dutyShoulder ; duty_cycle--){
          ledcWrite(PWM_CHANNEL_SHOULDER, duty_cycle);
          //delay(1);
        }
      }
      newDutyShoulder = false;
      prevDutyShoulder = dutyShoulder;

    }

    potValueShoulder = analogRead(27);
    duty_value_shoulder = ((((float)potValueShoulder/4095)*400) + 105);
    dutyShoulder = (int)round(duty_value_shoulder);
  
  }
  


