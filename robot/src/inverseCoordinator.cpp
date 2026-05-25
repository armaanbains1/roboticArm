#include <Arduino.h>
#include <iostream>
#include <cmath> 

using namespace std;

double pi = 3.14159265358979323846;
double rad_to_deg = 180 / pi;
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

//position movement variables
std::pair<double,double> prevCoord;
std::pair<double, double> coord;



// put function declarations here:


int inverseCalculateDutyShoulder(std::pair<double, double> coordinate){
    double dutyShoulder = 0;
    double degrees = 0;
    if (coordinate.first<0){
        cout << "smaller than 0" << endl;
        cout << atan2(coordinate.second, coordinate.first) << endl;
        degrees = ((atan2(coordinate.second, coordinate.first) * rad_to_deg)) - 90;
        dutyShoulder = ((degrees + 90.0) * 415.0 / 180.0) + 105.0;
    }
    else{
        cout << "greater than 0" << endl;
        cout << atan2(coordinate.second, coordinate.first) << endl;
        degrees = atan2(coordinate.second, coordinate.first) * rad_to_deg;
        dutyShoulder = ((degrees * 211.0) / 90.0) + 105.0;
    }
    return (int)dutyShoulder;
}


double calculateBicepLengthInv(double length, double zoffset){
  double angle = acos((pow(length, 2) + pow(10,2) - pow(16.5,2)) / (2*(length)*(10))) + atan((zoffset+7)/length);
  double lengthBicep = 10*cos(angle);
  return lengthBicep;
}

std::pair<double, double> calculateForearmLengthInv(double length, double zoffset){
  double angle = acos((pow(16.5, 2) + pow(length,2) - pow(10,2)) / (2*(length)*(16.5))) - atan((zoffset+7)/length);
  double lengthForearm = 16.5*cos(angle);
  std::pair<double, double> lengthAngle;
  lengthAngle.first = lengthForearm;
  lengthAngle.second = angle * (180.0 / 3.14159265358979323846);
  
  return lengthAngle;
}

int calculateBicepDuty(double bicepLength){
  int dutyBicep = (int)round((131.0 * ((90.0 - (acos(bicepLength / 10.0) / deg_to_rad)) / 90.0)) + 338.0);
  return dutyBicep;
}

int calculateForearmDuty(std::pair<double, double> lengthAngle){
  int dutyForearm = 0;
  if (lengthAngle.second >= 0){
    dutyForearm = (int)round((((acos(lengthAngle.first / 16.5) / deg_to_rad) / -30.0) * 111.0) + 223.0); 
  }
  else{
    dutyForearm = (int)round(223.0 - (((acos(lengthAngle.first / 16.5) / deg_to_rad) / 45.0) * 118.0));  
  }   
  return dutyForearm;
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

void moveToTarget(std::pair<double,double> coord, std::pair<double,double> prevCoord){
  
  if (coord != prevCoord){
    //cout << "atan result " << atan2(1, 1) * rad_to_deg << endl;
    cout << "new movement" << endl; 
    int dutyS =  inverseCalculateDutyShoulder(coord);
    int dutyB =  calculateBicepDuty(calculateBicepLengthInv(sqrt(pow(coord.first, 2) + pow(coord.second, 2)), 4));
    int dutyF = calculateForearmDuty(calculateForearmLengthInv(sqrt(pow(coord.first, 2) + pow(coord.second, 2)), 4));
    
    int prevDutyS =  inverseCalculateDutyShoulder(prevCoord);
    int prevDutyB =  calculateBicepDuty(calculateBicepLengthInv(sqrt(pow(prevCoord.first, 2) + pow(prevCoord.second, 2)), 4));
    int prevDutyF = calculateForearmDuty(calculateForearmLengthInv(sqrt(pow(prevCoord.first, 2) + pow(prevCoord.second, 2)), 4));
    
    if (prevDutyS >= dutyS)
        for (int duty_cycle = prevDutyS; duty_cycle >= dutyS ; duty_cycle--){
            ledcWrite(PWM_CHANNEL_SHOULDER, duty_cycle);
            delay(15);
        }
    else{
        for (int duty_cycle = prevDutyS; duty_cycle <= dutyS ; duty_cycle++){
            ledcWrite(PWM_CHANNEL_SHOULDER, duty_cycle);
            delay(15);
        }
    }

    if (prevDutyB >= dutyB)
        for (int duty_cycle = prevDutyB; duty_cycle >= dutyB ; duty_cycle--){
            ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
            delay(15);
        }
    else{
        for (int duty_cycle = prevDutyB; duty_cycle <= dutyB ; duty_cycle++){
            ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
            delay(15);
        }
    }

    if (prevDutyF >= dutyF)
        for (int duty_cycle = prevDutyF; duty_cycle >= dutyF ; duty_cycle--){
            ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
            delay(15);
        }
    else{
        for (int duty_cycle = prevDutyF; duty_cycle <= dutyF ; duty_cycle++){
            ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
            delay(15);
        }
    }
  }
    else{
        cout << "no new movement" << endl; 
    }
}


void setup() {
  Serial.begin(115200);
  bicepLength = 14.5;
  // put your setup code here, to run once:
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

  prevCoord.first = 20.0;
  prevCoord.second = 0.0;


  //120
  for (int duty_cycle = 0; duty_cycle <= 105 ; duty_cycle++){
    ledcWrite(PWM_CHANNEL_SHOULDER, duty_cycle);
    delay(15);
  }

  for (int duty_cycle = 0; duty_cycle <= 184 ; duty_cycle++){
    ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
    delay(15);
  }

  for (int duty_cycle = 0; duty_cycle <= 360 ; duty_cycle++){
    ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
    delay(15);
  }

  //delay(100);
/*
  for (int duty_cycle = 0; duty_cycle <= 25 ; duty_cycle++){
    ledcWrite(PWM_CHANNEL, duty_cycle);
    delay(1);
  }
*/

}

//std::pair<double, double> coordinateCalculator(){
    /*
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
  */
//}


/*
std::pair<double, double> inverseCoordinateCalculator(std::pair<double, double> coordinate){
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
*/

void loop(){
    
    if (Serial.available() > 0) {
            
            // Read the first two floating-point numbers separated by a space or comma
            double first = Serial.parseFloat();
            double second = Serial.parseFloat();

            // Clear any leftover newline characters (\r or \n) in the buffer
            while (Serial.available() > 0) {
                Serial.read();
            }

            coord.first = first;
            coord.second = second;
            
            // Print confirmations back to the serial monitor so you know it registered
            cout << "\n--- New Target Received ---" << endl;
            cout << "Target X: " << first << ", Y: " << second << endl;
            cout << " DUTY SHOULDER = " << inverseCalculateDutyShoulder(coord) << endl;
            cout << " DUTY BICEP = " << calculateBicepDuty(calculateBicepLengthInv(sqrt(pow(coord.first, 2) + pow(coord.second, 2)), 0)) << endl;
            cout << " DUTY FOREARM = " << calculateForearmDuty(calculateForearmLengthInv(sqrt(pow(coord.first, 2) + pow(coord.second, 2)), 0)) << endl;
            moveToTarget(coord, prevCoord);
            prevCoord = coord;
            

        }
    delay(50);
    
  }
  



