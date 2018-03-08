#ifndef balancing_h
#define balancing_h

#include "Arduino.h"

/* This code is based on Joop Brokkings excellent work:
  http://www.brokking.net/imu.html
  https://www.youtube.com/watch?v=4BoIE8YQwM8
  https://www.youtube.com/watch?v=j-kE0AMEWy4

  I (TheDIYGuy999) have modified it to fit my needs

  Now is Wu NOir who modifies it for his needs (^_^)/"

  Definizioni 
  Roll rotazione intorno ad asse X corrispondente alla direzione Nord
  Pitch rotazione intorno ad asse Y corrispondente alla direzione Est
  Yaw rotazione intorno ad asse Z corrispondente alla direzione verso giu/ gravita

  Sul mio robot 
  X corrisponde all asse Z 
  Y corrisponde all asse Y
  Z coorisponde alla asse -X

  This header file is required for the "balancing" (RobotType = 4) options in the RobotConfig.h
  Connect an MPU-6050 sensor to GND, VDD (3.3 and 5V compatible), SCL and SDA.
  Mount it as close as possible to the pivot point of your vehicle

  -->> Note:
  - The receiver will not work, if RobotType is set to 4 or 5 and no MPU-6050 sensor is wired up!!
  - !! Don't move your vehicle during gyro calibration! (about 6s after powering up) !!
  - The MPU-6050 requires about 20s to stabilize (finding the exact zero point) after powering on!
  - The measurements are taken with 125Hz (8ms) refresh rate. Reason: processing all the code requires up to
    7ms loop time with 8MHz MCU clock. --> You can measure your loop time with loopDuration()
*/

int Gyro_Address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int Acc_Calib_Value = 1000;                            //Enter the accelerometer calibration value trovat con YABR_hardware_test piazzando il robot vicino alla posizione di equilibrio


//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)
//
// =======================================================================================================
// GLOBAL VARIABLES
// =======================================================================================================
//

byte Start, received_byte, low_bat;

// 6050 variables
int Gyro_X, Gyro_Y, Gyro_Z;
long Acc_X_Raw, Acc_Y_Raw, Acc_Z_Raw;
long Acc_X, Acc_Y, Acc_Z, Acc_Tot_Vector;
int temperature;
//long loop_timer;
float Angle_Pitch, Angle_Roll;
boolean Set_Gyro_Angles;
float Angle_Roll_Acc, Angle_Pitch_Acc;
float Yaw_Rate;


int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

int speedAveraged;
int speedPot;

//Define PID Variables
double speedTarget, speedMeasured, speedOutput;
double angleTarget, angleMeasured, angleOutput;

double speedAverage;

// configuration variables (you may have to change them)
const int CalibrationPasses = 500; // 500 is useful

double i_timer=1000;

//
// =======================================================================================================
// PRINT DEBUG DATA
// =======================================================================================================
//

void writeDebug() {
#ifdef DEBUG
  static unsigned long lastPrint;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();

    Serial.print("P: ");
  //  Serial.print(Angle_Pitch);    //Print pitch
    Serial.print("   R: ");
 //   Serial.print(Angle_Roll);    //Print roll
    Serial.print("   Motor: ");
//    Serial.println(AngleOutput);    //Print Motor output
  }
#endif
}

//
// =======================================================================================================
// MPU 6050 SETUP
// =======================================================================================================
//

void setupMpu6050Register() {

  Wire.begin(sda,scl);                                                       //Start the I2C bus as master
  // Activate the MPU-6050
  Wire.beginTransmission(Gyro_Address);                                //Start communication with the MPU-6050
  Wire.write(0x6B);                                                    //We want to write to the PWR_MGMT_1 register (6B hex) // Send the requested starting register
  Wire.write(0x00);                                                    //Set the register bits as 00000000 to activate the gyro // Set the requested starting register
  Wire.endTransmission();                                              //End the transmission with the gyro.
  // Configure the accelerometer (+/-8g)
  Wire.beginTransmission(Gyro_Address);                                // Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //We want to write to the ACCEL_CONFIG register (1A hex) // Send the requested starting register
  Wire.write(0x10);                                                    //Set the register bits as 0x10 per +/- 8g se 0x08 sara 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                              //End the transmission with the gyro
  // Configure the gyro (2000°/s per second full scale)
  Wire.beginTransmission(Gyro_Address);                                // Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //We want to write to the GYRO_CONFIG register (1B hex) // Send the requested starting register
  Wire.write(0x18);                                                    //Set the register bits as 0x18 2000°/s per second full scale  se 0x00 00000000 (250°/s full scale)// Set the requested starting register
  Wire.endTransmission();                                              //End the transmission with the gyro.
  //Set some filtering to improve the raw data.
  // Wire.beginTransmission(Gyro_Address);                                     //Start communication with the address found during search
  // Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  // Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  // Wire.endTransmission();                                                   //End the transmission with the gyro 
  
  //pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  //pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  //pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  //pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
 // pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output
}

//
// =======================================================================================================
// PID SETUP
// =======================================================================================================
//

void setupPid() {

  // Speed control loop
  speedPid.SetSampleTime(8); // calcualte every 4ms = 250Hz
  speedPid.SetOutputLimits(-33, 33); // output range from -33 to 33 (same as in balancing() )
  speedPid.SetMode(AUTOMATIC);

  // Angle control loop
  anglePid.SetSampleTime(8); // calcualte every 4ms = 250Hz
  anglePid.SetOutputLimits(-43, 43); // output range from -43 to 43 for motor
  anglePid.SetMode(AUTOMATIC);
  
}



// Main function
void readMpu6050Data() {

  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(Gyro_Address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(Gyro_Address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += Acc_Calib_Value;                                //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;
                                                                            //devo ceccare se 8200 corrisponde a 9.81m/s2 cio l'accelerazione massima

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer 
                                                                            // angolo accelerometro=asin(data accelerometro/8200)*180/pi
 if (i_timer < millis()){
  i_timer =millis()+1000;                                                                         
 }
 
 if (millis() >= i_timer) {                                                                           
    Serial.print("angle_Acc = ");
    Serial.print(angle_acc);
 }
    
  if(Start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    Start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  Wire.beginTransmission(Gyro_Address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(Gyro_Address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  
  gyro_pitch_data_raw -= Gyro_Pitch_Calibration_Value;                      //Add the gyro calibration value (meno sensibile alle vibrazioni che l angolo derivato dall accelerazione)
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled angle during this loop and add this to the angle_gyro variable 
                                                                            // spec dicono che il valore del gyro a 131 corrisponde a 1°/s 
                                                                            // 1/131=x/4ms=x*250 -> 250*131*x=1°/s -> 1/(250x131)=0.000031 sono i gradi ogni 4ms
 

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= Gyro_Yaw_Calibration_Value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the 
                                                                            //accelerometer angle siccome col tempo il gyro angle drifta 
                                                                            // malgrado la sensibilita del angle_acc alle vibrazioni sia alta 
                                                                            //usare 0.0004 del suo valore per correggere il drift rende negligable le vibrazioni
                                                                            // lo si puo dunque usare in piccole dosi per eliminare il drift
 if (millis() >= i_timer) {
    Serial.print("   gyro_pitch_data_raw = ");
    Serial.print(gyro_pitch_data_raw);
    Serial.print("   angle_gyro = ");
    Serial.print(angle_gyro);
    
#ifdef ASD
  Serial.print("   ACC_X_Raw: ");
  Serial.print(Acc_X_Raw);
  Serial.print("   ACC_Y_Raw:: ");
  Serial.print(Acc_Y_Raw);
  Serial.print("   ACC_Z_Raw:: ");
  Serial.println(Acc_Z_Raw);
#endif

 }
 
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations       utilizzabile se il motore si comporta in modo lineare la parte coi 400 e per i suoi stepper motor io devo adattarla probabilmente
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || Start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    Start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop. Bisogna assicurarsi che il resto del code nel loop non eccede i 4 ms
  while(loop_timer > micros());
  loop_timer += 4000;
  
  //  readMpu6050Raw();                                                  // Read RAW data

  //  processMpu6050Data();                                              // Process the MPU 6050 data
}

#endif
