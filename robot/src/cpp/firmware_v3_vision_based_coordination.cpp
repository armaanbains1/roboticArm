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
int prevDutyH = 0;

int dutyS = 0;
int dutyB = 0;
int dutyF = 0;
int dutyH = 0;

int zoffset = 10;

int baselineForarm = 500;
int baselineHand = 500;

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
      cout << length << " is greater than " << max_reach;
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
      cout << length << " is greater than " << max_reach;
      cout << "WARNING: Target out of reach. Clamping forearm calculation." << endl;
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

int calculateHandDuty(bool greater, int newDutyForearm, int prevDutyForearm, bool isNew){
    int handAdjust = (newDutyForearm - prevDutyForearm) * 1.0;
    cout << "adjustment factor of " << handAdjust << endl;
    if (isNew){
      baselineHand -= handAdjust;
  }
  return baselineHand;
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
    for (int duty_cycle = 180; duty_cycle <= 380 ; duty_cycle++){
      ledcWrite(PWM_CHANNEL_GRIPPER, duty_cycle);
      //delay(0);
    }
}

void ungrip(){
    for (int duty_cycle = 380; duty_cycle >= 180 ; duty_cycle--){
      ledcWrite(PWM_CHANNEL_GRIPPER, duty_cycle);
      //delay(5); // ADDED DELAY HERE
    }
}

void moveToTarget(std::pair<double,double> coord, std::pair<double,double> prevCoord, int newZoffset){
  Serial.print(zoffset); Serial.print(F(" and new ")); Serial.println(newZoffset);
  
  if (coord != prevCoord || newZoffset != zoffset) {
    Serial.println(F("new movement")); 
        
    // 1. Calculate target duties
    dutyS = inverseCalculateDutyShoulder(coord);
    dutyB = calculateBicepDuty(calculateBicepLengthInv(coord, newZoffset));
    
    prevDutyS = inverseCalculateDutyShoulder(prevCoord);
    prevDutyB = calculateBicepDuty(calculateBicepLengthInv(prevCoord, zoffset));

    // Calculate forearm and hand duties based on direction
    if (dutyB >= prevDutyB) {
        prevDutyF = dutyF;
        dutyF = calculateForearmDuty(calculateForearmLengthInv(coord, newZoffset), true, dutyB, prevDutyB, true);
    } else {
        prevDutyF = dutyF;
        dutyF = calculateForearmDuty(calculateForearmLengthInv(coord, newZoffset), false, dutyB, prevDutyB, true);
    }

    if (dutyF >= prevDutyF) {
        prevDutyH = dutyH;
        dutyH = calculateHandDuty(true, dutyF, prevDutyF, true);
    } else {
        prevDutyH = dutyH;
        dutyH = calculateHandDuty(false, dutyF, prevDutyF, true);
    }

    zoffset = newZoffset;

    // 2. Determine the maximum number of steps required across ALL servos
    int deltaS = abs(dutyS - prevDutyS);
    int deltaB = abs(dutyB - prevDutyB);
    int deltaF = abs(dutyF - prevDutyF);
    int deltaH = abs(dutyH - prevDutyH);

    int maxSteps = max(max(deltaS, deltaB), max(deltaF, deltaH));

    if (maxSteps == 0) return; // No movement needed

    // 3. Coordinated step loop
    for (int step = 0; step <= maxSteps; step++) {
        float progress = (float)step / maxSteps; // Normalized progress from 0.0 to 1.0

        // Linearly interpolate current position for each servo
        int currentS = prevDutyS + (int)((dutyS - prevDutyS) * progress);
        int currentB = prevDutyB + (int)((dutyB - prevDutyB) * progress);
        int currentF = prevDutyF + (int)((dutyF - prevDutyF) * progress);
        int currentH = prevDutyH + (int)((dutyH - prevDutyH) * progress);

        // Write positions simultaneously
        ledcWrite(PWM_CHANNEL_SHOULDER, currentS);
        ledcWrite(PWM_CHANNEL_BICEP, currentB);
        ledcWrite(PWM_CHANNEL_FOREARM, currentF);
        ledcWrite(PWM_CHANNEL_HAND, currentH);

        delay(15); // Controls overall speed of the synchronized movement
    }

    // 4. Update tracking variables for debugging
    Serial.print(F("prevDutyB=")); Serial.print(prevDutyB); Serial.print(F(" dutyB=")); Serial.println(dutyB);
    Serial.print(F("prevDutyF=")); Serial.print(prevDutyF); Serial.print(F(" dutyF=")); Serial.println(dutyF);
  } else {
    Serial.println(F("no new movement")); 
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
  prevDutyS = 105; //changed this to 90 from 105 
  prevDutyH = 500;

  //120
  dutyS = 105;

  dutyB = calculateBicepDuty(calculateBicepLengthInv(prevCoord, zoffset));
  cout << "initial dutyB" << dutyB << endl;
  dutyF = calculateForearmDuty(calculateForearmLengthInv(prevCoord, zoffset), true, dutyB, prevDutyB, true);
  cout << "initial dutyF" << dutyF << endl;
  dutyH = 500;
  cout << "initial dutyF" << dutyH << endl;




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

  for (int duty_cycle = 0; duty_cycle <= 500 ; duty_cycle++){
      ledcWrite(PWM_CHANNEL_HAND, duty_cycle);
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
void loop() {
    // Only trigger if there is actual data waiting
    if (Serial.available() > 0) {
        
        // 1. Read the three floating-point numbers
        double first = Serial.parseFloat();
        double second = Serial.parseFloat();
        double third = Serial.parseFloat();

        // 2. Consume any whitespace/spaces between the last number and the command key
        while (Serial.available() > 0 && isspace(Serial.peek())) {
            Serial.read(); 
        }

        // 3. Read the keystroke safely
        char incomingKey = ' ';
        if (Serial.available() > 0) {
            incomingKey = Serial.read(); // Read the actual 'g' or 'u'
        }

        // Execute gripping logic
        if (incomingKey == 'g') {
            grip();
        } else if (incomingKey == 'u') {
            ungrip();
        }

        // 4. Properly clear out trailing newlines (\r or \n) so loop doesn't re-trigger
        delay(2); // Tiny delay to let trailing bytes finish arriving
        while (Serial.available() > 0) {
            Serial.read();
        }

        // Update coordinates
        coord.first = first;
        coord.second = second;
        coord.second += 2.5; 
        
        // 5. FIXED: Replaced 'cout' with Arduino 'Serial.print'
        Serial.println(F("\n--- New Target Received ---"));
        Serial.print(F("Target X: ")); Serial.print(first);
        Serial.print(F(", Y: ")); Serial.print(second);
        Serial.print(F(", Z: ")); Serial.println(third);
        
        int targetDutyB = calculateBicepDuty(calculateBicepLengthInv(coord, third));
        
        Serial.print(F(" DUTY SHOULDER = ")); Serial.println(inverseCalculateDutyShoulder(coord));
        Serial.print(F(" DUTY BICEP = ")); Serial.println(targetDutyB);
        
        if (dutyB <= targetDutyB) {
            Serial.print(dutyB); Serial.print(F(" TO ")); Serial.print(targetDutyB); Serial.println(F(" new is greater"));
        } else {
            Serial.print(dutyB); Serial.print(F(" TO ")); Serial.print(targetDutyB); Serial.println(F(" old is greater"));
        }

        // Safety Bounds Check
        if ((targetDutyB > 460) || (targetDutyB < 299)) {
            Serial.println(F("Issue: Target out of physical bounds!"));
        } else {
            moveToTarget(coord, prevCoord, third);
            prevCoord = coord;
        }  
    }
    
    delay(50);
}
  
