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
#ifndef MPU6050_NODE_H   //ifndef stands for "if not defined". The first pair of statements tells the program to define the TB6612FNG_DC_Motor_driver header file if it has not been defined already.
#define MPU6050_NODE_H

//#define DEBUGMPU // if not commented out, Serial.print() is active! For debugging only!!

#include <Wire.h>

// sensitivity scale factor respective to full scale setting provided in datasheet 
#define AccelScaleFactor 16384
#define GyroScaleFactor  131

// MPU6050 few configuration register addresses
#define MPU6050_REGISTER_SMPLRT_DIV  		0x19
#define MPU6050_REGISTER_USER_CTRL   		0x6A
#define MPU6050_REGISTER_PWR_MGMT_1   		0x6B
#define MPU6050_REGISTER_PWR_MGMT_2   		0x6C
#define MPU6050_REGISTER_CONFIG      		0x1A
#define MPU6050_REGISTER_GYRO_CONFIG  		0x1B
#define MPU6050_REGISTER_ACCEL_CONFIG 		0x1C
#define MPU6050_REGISTER_FIFO_EN      		0x23
#define MPU6050_REGISTER_INT_ENABLE   		0x38
#define MPU6050_REGISTER_ACCEL_XOUT_H 		0x3B
#define MPU6050_REGISTER_SIGNAL_PATH_RESET	0x68
#define LOOP_TIME 4000   // era inizialmente 4000 per 4ms ma non basta lo passo a 8000 a 10000


class MPU6050 
{

public:
	MPU6050 ( const uint8_t sdaPin, const uint8_t sclPin, const uint8_t slaveAddress ); // constructor
	void Setup_NODE_Register(); 
//
// =======================================================================================================
// READ MPU 6050 RAW DATA FUNCTION
// =======================================================================================================
//
	void Read_Print_Value() ;
	void Calibrate();
//
// =======================================================================================================
// PROCESS MPU 6050 DATA SUBFUNCTION
// =======================================================================================================
//
	void ProcessData();

  double Angle_Roll = 0; 
  double Angle_Pitch = 0; 
  double Angle_Yaw = 0;      //prima era long e il Node faceva un overflow
  
private:

// read all 14 register
	void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress);

	void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);

// MPU6050 Slave Device Address
	const uint8_t MPU6050SlaveAddress;

	// Select SDA and SCL pins for I2C communication 
	const uint8_t scl;
	const uint8_t sda;

	int16_t AccelX = 0;
	int16_t AccelY = 0;
	int16_t AccelZ = 0;
	int16_t Temperature = 0;
	int16_t GyroX = 0;
	int16_t GyroY = 0;
	int16_t GyroZ = 0;


	double Ax = 0;
	double Ay = 0;
	double Az = 0;
	double T = 0;
	double Gx = 0;
	double Gy = 0;
	double Gz = 0;

	const int CalibrationPasses = 500; // 500 is useful
	/*long Gyro_X_calib = 0; 
	long Gyro_Y_calib = 0; 
	long Gyro_Z_calib = 0;
	long Acc_X_calib = 0; 
	long Acc_Y_calib = 0; 
	long Acc_Z_calib = 0;
	*/
	double Gyro_X_calib = 0; 
	double Gyro_Y_calib = 0; 
	double Gyro_Z_calib = 0;
	double Acc_X_calib = 0; 
	double Acc_Y_calib = 0; 
	double Acc_Z_calib = 0;


	double Acc_Tot_Vector = 0;
	double Angle_Roll_Acc = 0;
	double Angle_Pitch_Acc = 0;
	//double Angle_Yaw_Acc = 0;      // l'Acc Yaw non esiste perche per dedurre l'angolo dal vettore totale richiede un angolo rispetto alla gravita
									//e lo yaw si trova sul vettore della gravita nel caso del robot
	bool Set_Gyro_Angles;

	unsigned long Calib_loop_timer=0;
	//double Calib_loop_timer=0;
	//double LOOP_TIME = 8000;  // era inizialmente 4000 per 4ms ma non basta lo passo a 8000 a 10000
	unsigned long Loop_time_ms=(LOOP_TIME/1000)-1;
	double i_timer=1000;

	double CPUFrequency = 80;  // 80Mhz for NodeMCU ESP8266 , Arduino is 8 or 16Mhz



};
#endif // ends the condition #ifndef MPU6050_NODE_H

