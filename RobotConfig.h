#ifndef RobotConfig_h
#define RobotConfig_h

#include "Arduino.h"

#define ROBOT_NODEMCU_BALANCING // <- Select the correct vehicle configuration here before uploading!

//
// =======================================================================================================
// ROBOT SPECIFIC CONFIGURATIONS
// =======================================================================================================
//

/*


  // Robot type
  byte RobotType;
  0 = NodeMCU Self Balancing Robot
  
  
  
  // Motor configuration
  int maxPWMfull; // (100% PWM is 255)
  int maxPWMlimited;
  int minPWM; // backlash compensation for self balancing applications
  byte maxAccelerationFull;// (ms per 1 step input signal change)
  byte maxAccelerationLimited;

  // Variables for self balancing (vehicleType = 4) only!
  float tiltCalibration = -0.2; // -0.2° (+ = leans more backwards!) Vary a bit, if you have slow oscillation with big amplitude
*/



#ifdef ROBOT_NODEMCU_BALANCING

// Robot type
byte RobotType = 0;  //0 = NodeMCU Self Balancing Robot

// Motor configuration
int maxPWMfull = 255;
int maxPWMlimited = 170;
int minPWM = 15; // 15 Backlash compensation, important for self balancing!
byte maxAccelerationFull = 3;
byte maxAccelerationLimited = 12;

// Variables for self balancing (vehicleType = 4) only!
float tiltCalibration = -0.2; // -0.2° (+ = leans more backwards!) Vary a bit, if you have slow oscillation with big amplitude

#endif

#endif