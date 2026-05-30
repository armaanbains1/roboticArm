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

int prevDutyS = 0;
int prevDutyB = 0;
int prevDutyF = 0;

int dutyS = 0;
int dutyB = 0;
int dutyF = 0;

int zoffset = 10;

int baselineForarm = 500;

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


double calculateBicepLengthInv(std::pair<double, double> coord, double zoffset){
  // 9 = from height of gripper
  // 4 = height from bottom of base
  // 2 = height from item.
  double ground_dist = sqrt(pow(coord.first, 2) + pow(coord.second, 2));
  double length = sqrt(pow(ground_dist, 2) + pow(zoffset-5, 2));

  double max_reach = 29.99; // 13.5 (bicep) + 16.5 (forearm) - 0.01 to prevent floating point errors
  if (length > max_reach) {
      cout << "WARNING: Target out of reach. Clamping bicep calculation." << endl;
      length = max_reach;
  }

  double angle = acos((pow(length, 2) + pow(13.5,2) - pow(16.5,2)) / (2*(length)*(13.5))) + atan((zoffset-5)/ground_dist);
  double lengthBicep = 13.5*cos(angle);
  return lengthBicep;
}

std::pair<double, double> calculateForearmLengthInv(std::pair<double, double> coord, double zoffset){
  double ground_dist = sqrt(pow(coord.first, 2) + pow(coord.second, 2));
  double length = sqrt(pow(ground_dist, 2) + pow(zoffset-5, 2));

  double max_reach = 29.99;
  if (length > max_reach) {
      cout << "WARNING: Target out of reach. Clamping bicep calculation." << endl;
      length = max_reach;
  } 

  double angle = acos((pow(16.5, 2) + pow(length,2) - pow(13.5,2)) / (2*(length)*(16.5))) - atan((zoffset-5)/ground_dist);
  double lengthForearm = 16.5*cos(angle);
  std::pair<double, double> lengthAngle;
  lengthAngle.first = lengthForearm;
  lengthAngle.second = angle * (180.0 / 3.14159265358979323846);
  
  return lengthAngle;
}

int calculateBicepDuty(double bicepLength){
  int dutyBicep = (int)round((131.0 * ((90.0 - (acos(bicepLength / 13.5) / deg_to_rad)) / 90.0)) + 338.0);
  return dutyBicep;
}




int calculateForearmDuty(std::pair<double, double> lengthAngle, bool greater, int newDutyBicep, int prevDutyBicep, bool isNew){
  int dutyForearm = 0;
  int forearmAdjust = (newDutyBicep - prevDutyBicep) * 2.073;
  cout << "adjustment factor of " << forearmAdjust << endl;
  if (isNew){
    baselineForarm -= forearmAdjust;
  }

  if (lengthAngle.second <= 0){
    dutyForearm = (int)round((((acos(lengthAngle.first / 16.5) / deg_to_rad) / -30.0) * 111.0) - (baselineForarm)); 
  }
  else{
    dutyForearm = (int)round((baselineForarm) + (((acos(lengthAngle.first / 16.5) / deg_to_rad) / 45.0) * 118.0));  
  }   
  
  
  cout << " new forearm duty " << dutyForearm;
  return dutyForearm;
}


double calculateBicepLength(){

  if (dutyBicep >= 338){
    bicepLength = 13.5 * cos(((double)(90.0 -(90.0*(double)(((dutyBicep - 338.0) / (469.0 - 338.0))))))*deg_to_rad);
  }
  else if (dutyBicep < 338){
    bicepLength = -13.5 * cos(((double)((dutyBicep/338.0)) * 90.0)*deg_to_rad);
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
        
    dutyS =  inverseCalculateDutyShoulder(coord);
    dutyB =  calculateBicepDuty(calculateBicepLengthInv(coord, zoffset));
    cout << "cur" << dutyB << endl;

    prevDutyS =  inverseCalculateDutyShoulder(prevCoord);

    prevDutyB =  calculateBicepDuty(calculateBicepLengthInv(prevCoord,zoffset));
    cout << "prev" << prevDutyB << endl;


    if (dutyB >= prevDutyB){
        prevDutyF = dutyF;
        dutyF = calculateForearmDuty(calculateForearmLengthInv(coord, zoffset), true, dutyB, prevDutyB, true);
        // FIXED: Using prevDutyB to calculate the true previous starting point
        //prevDutyF = calculateForearmDuty(calculateForearmLengthInv(prevCoord, 10), true, prevDutyB, false);
    }
    else{
        prevDutyF = dutyF;
        dutyF = calculateForearmDuty(calculateForearmLengthInv(coord, zoffset), false, dutyB, prevDutyB, true);
        // FIXED: Using prevDutyB to calculate the true previous starting point
        //prevDutyF = calculateForearmDuty(calculateForearmLengthInv(prevCoord, 10), false, prevDutyB, false);
    }
    
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
    cout << "prevDutyB=" << prevDutyB << " dutyB=" << dutyB << endl;
    cout << "prevDutyF=" << prevDutyF << " dutyF=" << dutyF << endl;
    cout << "forearmAdjust=" << (dutyB - prevDutyB) * 2.073 << endl;
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


  ledcSetup(PWM_CHANNEL_FOREARM, FREQ, PWM_RESOLUTION);
  ledcAttachPin(19, PWM_CHANNEL_FOREARM);


  ledcSetup(PWM_CHANNEL_SHOULDER, FREQ, PWM_RESOLUTION);
  ledcAttachPin(5, PWM_CHANNEL_SHOULDER); 

  ledcSetup(PWM_CHANNEL_HAND, FREQ, PWM_RESOLUTION);
  ledcAttachPin(4, PWM_CHANNEL_HAND); 


  ledcSetup(PWM_CHANNEL_GRIPPER, FREQ, PWM_RESOLUTION);
  ledcAttachPin(21, PWM_CHANNEL_GRIPPER); 

  prevCoord.first = 20.0;
  prevCoord.second = 2.5;
  prevDutyB = 338;
  prevDutyF = 500;
  prevDutyS = 105;
  //120
  dutyS = 105;

  dutyB = calculateBicepDuty(calculateBicepLengthInv(prevCoord, zoffset));
  cout << "initial dutyB" << dutyB << endl;
  dutyF = calculateForearmDuty(calculateForearmLengthInv(prevCoord, zoffset), true, dutyB, prevDutyB, true);
  cout << "initial dutyF" << dutyF << endl;

// starting point (20,0)

if (prevDutyB >= dutyB) {
      for (int duty_cycle = prevDutyB; duty_cycle >= dutyB ; duty_cycle--){
          ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
          delay(15);
      }
  } else {
      for (int duty_cycle = prevDutyB; duty_cycle <= dutyB ; duty_cycle++){
          ledcWrite(PWM_CHANNEL_BICEP, duty_cycle);
          delay(15);
      }
  }

  // --- Forearm Startup Movement ---
  if (prevDutyF >= dutyF) {
      for (int duty_cycle = prevDutyF; duty_cycle >= dutyF ; duty_cycle--){
          ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
          delay(15);
      }
  } else {
      for (int duty_cycle = prevDutyF; duty_cycle <= dutyF ; duty_cycle++){
          ledcWrite(PWM_CHANNEL_FOREARM, duty_cycle);
          delay(15);
      }
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
            coord.second += 2.5;
            
            // Print confirmations back to the serial monitor so you know it registered
            cout << "\n--- New Target Received ---" << endl;
            cout << "Target X: " << first << ", Y: " << second << endl;
          // FIXED: Calculate a temporary target bicep value so the prints are honest
            int targetDutyB = calculateBicepDuty(calculateBicepLengthInv(coord, zoffset));
            
            cout << " DUTY SHOULDER = " << inverseCalculateDutyShoulder(coord) << endl;
            cout << " DUTY BICEP = " << targetDutyB << endl;
            
            if (dutyB <= targetDutyB){
                cout << dutyB << " TO " << targetDutyB << "new is greater" << endl;
                //cout << " DUTY FOREARM = " << calculateForearmDuty(calculateForearmLengthInv(coord, 10), true, targetDutyB) << endl;
            }
            else{
                cout << dutyB << " TO " << targetDutyB << "old is gfreater" << endl;
                //cout << " DUTY FOREARM = " << calculateForearmDuty(calculateForearmLengthInv(coord, 10), false, targetDutyB) << endl;
            }
            if ((targetDutyB > 460) || (targetDutyB < 299)){
                cout << "Issue";
            }
            else{
                moveToTarget(coord, prevCoord);
                prevCoord = coord;
            }  

        }
    delay(50);
    
    
  }
  
