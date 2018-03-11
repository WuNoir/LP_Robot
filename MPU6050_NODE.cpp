/**
* 
Interfacing MPU6050 with NODE
Exemple taken from http://www.electronicwings.com/nodemcu/mpu6050-interfacing-with-nodemcu
and modified for balancing robot

 Definizioni 
 Roll rotazione intorno ad asse X corrispondente alla direzione Nord
 Pitch rotazione intorno ad asse Y corrispondente alla direzione Est
 Yaw rotazione intorno ad asse Z corrispondente alla direzione verso giu/ gravita

 Sul mio robot (Robot 0)
 X corrisponde all asse Z 
 Y corrisponde all asse Y
 Z coorisponde alla asse -X


* @author Lorenzo Piccardi
*/
#include <Arduino.h>
#include "MPU6050_NODE.h"


MPU6050::MPU6050(const uint8_t sdaPin, const uint8_t sclPin, const uint8_t slaveAddress) : 
sda(sdaPin), scl(sclPin), MPU6050SlaveAddress(slaveAddress)
{
};

void MPU6050::I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
 Wire.beginTransmission(deviceAddress);
 Wire.write(regAddress);
 Wire.write(data);
 Wire.endTransmission();
}


//void setup() {
void MPU6050::Setup_NODE_Register() {
//  Serial.begin(11520);
 Wire.begin(sda, scl);
//configure MPU6050 , Initialisation
 delay(150);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);    //0x68, 0x19
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);    //0x68, 0x6B
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);    //0x68, 0x6C
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);    //0x68, 0x1A
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale        //0x68, 0x1B
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale    //0x68, 0x1C
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);    //0x68, 0x23
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);    //0x68, 0x38
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);    //0x68, 0x68
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);    //0x68, 0x6A
}

//
// =======================================================================================================
// READ MPU 6050 RAW DATA FUNCTION
// =======================================================================================================
//

// read all 14 register
void MPU6050::Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
 Wire.beginTransmission(deviceAddress);
 Wire.write(regAddress);
 Wire.endTransmission();
 Wire.requestFrom(deviceAddress, (uint8_t)14);
 AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
 AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
 AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
 Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
 GyroX = (((int16_t)Wire.read()<<8) | Wire.read());              //Roll   nel robot balanging Yaw -> GiroX per girare
 GyroY = (((int16_t)Wire.read()<<8) | Wire.read());              //Pitch  nel robot balanging Pitch -> GiroY per balancare
 GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());              //Yaw    nel robot balanging Roll  -> GiroZ e un grado di liberta che non mi serve
   
 }


//void loop () {
void MPU6050::Read_Print_Value() {
 Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H); // 0x68, 0x3B
 
 //divide each with their sensitivity scale factor
 Ax = (double)AccelX/AccelScaleFactor;
 Ay = (double)AccelY/AccelScaleFactor;
 Az = (double)AccelZ/AccelScaleFactor;
 T = (double)Temperature/340+36.53; //temperature formula
 Gx = (double)GyroX/GyroScaleFactor;
 Gy = (double)GyroY/GyroScaleFactor;
 Gz = (double)GyroZ/GyroScaleFactor;

 Serial.print("Ax: "); Serial.print(Ax);
 Serial.print(" Ay: "); Serial.print(Ay);
 Serial.print(" Az: "); Serial.print(Az);
 Serial.print(" T: "); Serial.print(T);
 Serial.print(" Gx: "); Serial.print(Gx);
 Serial.print(" Gy: "); Serial.print(Gy);
 Serial.print(" Gz: "); Serial.println(Gz);

// delay(100);
}

void MPU6050::Calibrate(){
 
 // Calibrate the gyro (the vehicle must stay steady during this time!)
 delay(1500);                                                         // Delay 1.5 seconds to display the text
#ifdef DEBUG
 Serial.println("Calibrating gyro");                                  // Print text to console
#endif

 int cal_int = 0; 
 double inizio_calib = micros();    
 double lastGyroCal;
 while (cal_int < CalibrationPasses) {                                // Run the calibrating code X times , 500 volte

   if (micros() - lastGyroCal >= LOOP_TIME-1700) {              //-1700 che non si sa da dove arriva // Read the data every 8000us (equals 125Hz)
#ifdef DEBUG
     if (cal_int % (CalibrationPasses / 15) == 0)Serial.print(".");   // Print a dot every X readings
#endif
      
     Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H); // 0x68, 0x3B // Read the raw acc and gyro data from the MPU-6050
     Gyro_X_calib += GyroX;                                            // Add the gyro x-axis offset to the gyro_x_cal variable
     Gyro_Y_calib += GyroY;                                            // Add the gyro y-axis offset to the gyro_y_cal variable
     Gyro_Z_calib += GyroZ;                                            // Add the gyro z-axis offset to the gyro_z_cal variable
     Acc_X_calib += AccelX;                                            // Add the gyro x-axis offset to the gyro_x_cal variable
     Acc_Y_calib += AccelY;                                            // Add the gyro y-axis offset to the gyro_y_cal variable
     Acc_Z_calib += AccelZ;                                            // Add the gyro z-axis offset to the gyro_z_cal variable

     //delayMicroseconds(LOOP_TIME-300);                                                //Wait for 3700 microseconds to simulate the main program loop time  4000us => 250Hz
     //delayMicroseconds(LOOP_TIME); // simulate the main program loop time ??
      cal_int ++;
      delay(0.3);
      lastGyroCal=micros();
   }
     //delay(Loop_time_ms);     // x ms forse il delay aiuta allo stack overflow? strano  Aggiustarlo in base alla lunghezza del loop calib misurato
     //delay(5);                     // 5 ms forse il delay aiuta allo stack overflow? strano  Aggiustarlo in base alla lunghezza del loop calib misurato
     
 //if(Calib_loop_timer <= micros()) Serial.println("ERROR Calib loop too short !");
 }
 
 double fine_calib = micros();
 double durata_loop_calib = (fine_calib - inizio_calib)/CalibrationPasses;///1000;
 Serial.println("Loop calib finito ");
 Serial.print("Durata Loop Calib ms "); Serial.println(durata_loop_calib);
 
 if (durata_loop_calib > LOOP_TIME){
 Serial.println("LOOP TIME TOO SHORT ");   
   }
 
 Gyro_X_calib /= CalibrationPasses;                                      // Divide the gyro_x_cal variable by X to get the avarage offset
 Gyro_Y_calib /= CalibrationPasses;                                      // Divide the gyro_y_cal variable by X to get the avarage offset
 Gyro_Z_calib /= CalibrationPasses;                                      // Divide the gyro_z_cal variable by X to get the avarage offset
 Acc_X_calib /= CalibrationPasses;                                      // Divide the gyro_x_cal variable by X to get the avarage offset
 Acc_Y_calib /= CalibrationPasses;                                      // Divide the gyro_y_cal variable by X to get the avarage offset
 Acc_Z_calib /= CalibrationPasses;                                      // Divide the gyro_z_cal variable by X to get the avarage offset


#ifdef DEBUG

 Serial.print("GX calib: ");
 Serial.print(Gyro_X_calib);
 Serial.print("   GY calib: ");
 Serial.print(Gyro_Y_calib);
 Serial.print("   GZ calib: ");
 Serial.println(Gyro_Z_calib);
 Serial.print("AccX calib: ");
 Serial.print(Acc_X_calib);
 Serial.print("   AccY calib: ");
 Serial.print(Acc_Y_calib);
 Serial.print("   AccZ calib: ");
 Serial.println(Acc_Z_calib);
 Serial.println("done!");
     delay(4000); 
#endif
}


//
// =======================================================================================================
// PROCESS MPU 6050 DATA SUBFUNCTION
// =======================================================================================================
//

void MPU6050::ProcessData() {
 Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H); // 0x68, 0x3B
 Gx = GyroX - Gyro_X_calib;                                                // Subtract the offset calibration value from the raw gyro_x value
 Gy = GyroY - Gyro_Y_calib;                                                // Subtract the offset calibration value from the raw gyro_y value
 Gz = GyroZ - Gyro_Z_calib;                                                // Subtract the offset calibration value from the raw gyro_z value
 //Ax = AccelX - Acc_X_calib;                                                // Subtract the offset calibration value from the raw gyro_x value
 //Ay = AccelY - Acc_Y_calib;                                                // Subtract the offset calibration value from the raw gyro_y value
 //Az = AccelZ - Acc_Z_calib;                                                // Subtract the offset calibration value from the raw gyro_z value
// Ax = AccelX/AccelScaleFactor;
//  Ay = AccelY/AccelScaleFactor;          
// Az = AccelZ/AccelScaleFactor;                                               //divide each with their sensitivity scale factor
 Ax = AccelX;
 Ay = AccelY;          
 Az = AccelZ;  
 
 // accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
// accelerometer_data_raw += Acc_Calib_Value;                                //Add the accelerometer calibration value
  if(Ax > AccelScaleFactor)Ax = AccelScaleFactor;           //Prevent division by zero by limiting the acc data to +/-16384;
  if(Ax < -AccelScaleFactor)Ax = -AccelScaleFactor;         //Prevent division by zero by limiting the acc data to +/-16384;
  if(Ay > AccelScaleFactor)Ay = AccelScaleFactor;           //Prevent division by zero by limiting the acc data to +/-16384;
  if(Ay < -AccelScaleFactor)Ay = -AccelScaleFactor;         //Prevent division by zero by limiting the acc data to +/-16384;
  if(Az > AccelScaleFactor)Az = AccelScaleFactor;           //Prevent division by zero by limiting the acc data to +/-16384;
  if(Az < -AccelScaleFactor)Az = -AccelScaleFactor;         //Prevent division by zero by limiting the acc data to +/-16384;
                                                                           //devo ceccare se 8200 corrisponde a 9.81m/s2 cio l'accelerazione massima

#ifdef BALANCING    //**********************************************
 if (RobotType == 0) {  // Those calculations are only required for the self balancing robot. Otherwise we can save some processing time.
   
 }
// spec dicono che il valore del gyro a 131 corrisponde a 1°/s 
// 1/131=x/4ms=x*250 -> 250*131*x=1°/s -> 1/(250x131)=1/32750=0.000031 sono i gradi ogni 4ms se uso LOOP_TIME 1/131=x/LOOP_TIMEms->x=LOOP_TIME/131/1000000
 //Angle_Roll += (Gz * LOOP_TIME/131/1000000);   // (Gz/32750) Calculate the traveled roll angle and add this to the angle_roll variable
 Angle_Pitch += (Gy * LOOP_TIME/131/1000000);    // (Gy/32750)Calculate the traveled pitch angle and add this to the Angle_Pitch variable
 Angle_Yaw += (Gx * LOOP_TIME/131/1000000);      // (Gx/32750)Yaw rate in degrees per second
#endif

 if (i_timer < millis()){
 i_timer =millis()+1000;                                                                         
}

if (millis() >= i_timer) {                                                                           

 //Serial.print(" Angle_Roll: ");          //normalmente nel robot balancing non cambia perche non e un suo grado di liberta
// Serial.print(Angle_Roll);
 Serial.print(" Angle_Pitch: ");
 Serial.print(Angle_Pitch);
 Serial.print("  Angle_Yaw: ");
 Serial.println(Angle_Yaw);
 Serial.print("   Time: ");
 Serial.println(micros()/1000000); 

#ifdef DEBUGMPU  
#ifdef DEBUG

 Serial.print("   GX : ");
 Serial.print(Gx);
 Serial.print("   GY : ");
 Serial.print(Gy);
 Serial.print("   GZ : ");
 Serial.print(Gz);

 Serial.print("  Ax: ");
 Serial.print(Ax);
 Serial.print("   Ay: ");
 Serial.print(Ay);
 Serial.print("   Az: ");
 Serial.println(Az);
 Serial.print("   Time: ");
 Serial.println(micros()/1000000);

#endif
#endif
}
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians (not required in this application)
 //TheDIYGuy999      Angle_Pitch += Angle_Roll * sin(gyro_z * 0.000001066);               // If the IMU has yawed transfer the roll angle to the pitch angle
 //TheDIYGuy999      Angle_Roll -= Angle_Pitch * sin(gyro_z * 0.000001066);               // If the IMU has yawed transfer the pitch angle to the roll angle

   //Accelerometer reading averaging to improve vibration resistance (added by TheDIYGuy999)
   //Acc_X = (Acc_X * 9 + Acc_X_Raw) / 10;
   //Acc_Y = (Acc_Y * 9 + Acc_Y_Raw) / 10;
   //Acc_Z = (Acc_Z * 9 + Acc_Z_Raw) / 10;

   
   // 1/16384=x/4ms=x*250 -> 250*16384*x=g -> 1/(250x16384)=1/4096000=0.000000244140625 sono i g? ogni 4ms   AccelScaleFactor = 16384;  +/- 2g full scale 
   //Accelerometer angle calculations
   Acc_Tot_Vector = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az)); // Calculate the total accelerometer vector
  // Acc_Tot_Vector = AccelScaleFactor;                          //vettore gravita uguale a 16384
   //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
   if (abs(Az) < Acc_Tot_Vector) {                                      //Prevent the asin function to produce a NaN Prevent division by zero 
     Angle_Pitch_Acc = asin((float)Az / Acc_Tot_Vector) * 180/3.142;       //Calculate the pitch angle.
   }
 /*  if (abs(Ay) < Acc_Tot_Vector) {                                      //Prevent the asin function to produce a NaN Prevent division by zero 
     Angle_Roll_Acc = asin((float)Ay / Acc_Tot_Vector) * -180/3.142;       //Calculate the roll angle.
   }
 */  

#ifdef DEBUGMPU  
if (millis() >= i_timer) {  
 Serial.print("  Acc_Tot_Vector: ");
 Serial.print(Acc_Tot_Vector);
 Serial.print("  Angle_Pitch_Acc: ");
 Serial.print(Angle_Pitch_Acc);
 Serial.print("  Angle_Roll_Acc: ");
 Serial.println(Angle_Roll_Acc);
 }
#endif

   if (Set_Gyro_Angles) {                                               // If the IMU is already started (zero drift protection)
     Angle_Pitch = Angle_Pitch * 0.99 + Angle_Pitch_Acc * 0.01;         // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  //   Angle_Roll = Angle_Roll * 0.99 + Angle_Roll_Acc * 0.01;            // Correct the drift of the gyro roll angle with the accelerometer roll angle
                                                                        // l'Acc Yaw non esiste perche per dedurre l'angolo dal vettore totale richiede un angolo rispetto alla gravita
                                                                        //e lo yaw si trova sul vettore della gravita nel caso del robot
   }
   else {                                                               // At first start
     Angle_Pitch = Angle_Pitch_Acc;                                     // Set the gyro pitch angle equals to the accelerometer pitch angle
 //    Angle_Roll = Angle_Roll_Acc;                                       // Set the gyro roll angle equals to the accelerometer roll angle
     Set_Gyro_Angles = true;                                            // Set the IMU started flag
   }
  
if (millis() >= i_timer) {  
 Serial.print(" Angle_Pitch: ");
 Serial.println(Angle_Pitch);
//  Serial.print("  Angle_Roll: ");
// Serial.println(Angle_Roll);
}

 /*
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //Loop time timer
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
 //is created by setting the loop_timer variable to (+4000) LOOP_TIME microseconds every loop. 
 //Bisogna assicurarsi che il resto del code nel loop non eccede i 4 ms
 if(loop_timer <= micros()) Serial.println("ERROR loop too short !");
 while(loop_timer > micros());
 loop_timer += LOOP_TIME;
 */
}


