/**
 * @file TB6612FNG_DC_Motor_motor_driver.h
 * @brief Motor device driver for the TB6612FNG DC Motor driver

VM	Motor Voltage	Power	This is where you provide power for the motors (2.2V to 13.5V)
VCC	Logic Voltage	Power	This is the voltage to power the chip and talk to the microcontroller (2.7V to 5.5V)
GND	Ground	Power	Common 	Ground for both motor voltage and logic voltage (all GND pins are connected)
STBY	Standby	Input		Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
AIN1/BIN1	Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
AIN2/BIN2	Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
PWMA/PWMB	PWM input for channels A/B	Input	PWM input that controls the speed
A01/B01	Output 1 for channels A/B	Output	One of the two outputs to connect the motor
A02/B02	Output 2 for channels A/B	Output	One of the two outputs to connect the motor

In1		In2		PWM		Out1	Out2	Mode
H		H		H/L		L		L		Short brake
L		H		H		L		H		CCW
L		H		L		L		L		Short brake
H		L		H		H		L		CW
H		L		L		L		L		Short brake
L		L		H		OFF		OFF		Stop
Don’t forget STBY must be high for the motors to drive.


 * @author Lorenzo Piccardi
 */
#ifndef TB6612FNG_DC_Motor_driver_H		//ifndef stands for "if not defined". The first pair of statements tells the program to define the TB6612FNG_DC_Motor_driver header file if it has not been defined already.
#define TB6612FNG_DC_Motor_driver_H
//#define DEBUG_TB       // if not commented out, Serial.print() is active! For debugging only!!


class MyMotor
{
public:
MyMotor(int ain1, int ain2, int pwma, int offsetA, int stby)		//constructor      (pin assegnato a in1 o in3, pin assegnato a in2 o in4, pin assegnato a ena o enb)
		: AIN1oBIN1(ain1), AIN2oBIN2(ain2), PWMAoPWMB(pwma), OFFSETA(offsetA), STBY(stby)  // valori dichiarati piu sotto come privati che sono poi utilizzati all interno della classe MyMotor
	{}

	void	initialize()      //inizializzazione dei pin in OUTPUT che pilotano il motore da chiamare nel void setup nel file ino principale
	{
		pinMode(AIN1oBIN1,OUTPUT);
		pinMode(AIN2oBIN2,OUTPUT);
		pinMode(PWMAoPWMB,OUTPUT);
		pinMode(STBY,OUTPUT);
		digitalWrite(AIN1oBIN1, LOW);	//aggiunti come da TheDIYGuy999 per mettersi in mode Stop con gli output a OFF
		digitalWrite(AIN2oBIN2, LOW);	//aggiunti come da TheDIYGuy999 per mettersi in mode Stop con gli output a OFF
		digitalWrite(PWMAoPWMB, HIGH);	//aggiunti come da TheDIYGuy999 per mettersi in mode Stop con gli output a OFF
	}


	void	moveforward(int SPEED)  //velocita SPEED, PWM con analogwrite sui pin PWMA o PWMB ,    L		H		H		L		H		CCW forward
	{
		digitalWrite(AIN1oBIN1,LOW);
		digitalWrite(AIN2oBIN2,HIGH);
		analogWrite(PWMAoPWMB,SPEED);
		digitalWrite(STBY,HIGH);
    #ifdef DEBUG_TB
		Serial.println("go newforward!");
    #endif
	}
	
	void	movebackward(int SPEED)  //velocita SPEED, PWM con analogwrite sui pin PWMA o PWMB ,    H		L		H		H		L		CW backward
	{
		digitalWrite(AIN1oBIN1,HIGH);
		digitalWrite(AIN2oBIN2,LOW);
		analogWrite(PWMAoPWMB,SPEED);
		digitalWrite(STBY,HIGH);
    #ifdef DEBUG_TB
		Serial.println("go newbackward!");
    #endif
	}
	
	void	stop()
	{
		digitalWrite(AIN1oBIN1,LOW);
		digitalWrite(AIN2oBIN2,LOW);
		analogWrite(PWMAoPWMB,LOW);
		digitalWrite(STBY,LOW);  //    L		L		H		OFF		OFF		Stop
    #ifdef DEBUG_TB
		Serial.println("newStop!");
    #endif
	}
	
//~MyMotor();		// destructor  // commentato perche il compilatore non gli piace il destructor penso che l utilizzo sbagliato

private:
int AIN1oBIN1;
int AIN2oBIN2;
int PWMAoPWMB;
int OFFSETA;
int STBY;
};	


#endif // ends the condition #ifndef TB6612FNG_DC_Motor_driver_H 
