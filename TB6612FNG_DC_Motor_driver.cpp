/* Implentation of the class Motor*/
#include TB6612FNG_DC_Motor_driver.h


TB6612Motor::TB6612Motor(int in1, int in2, int pwm, int offset, int stby)		//constructor      (pin assegnato a in1 o in3, pin assegnato a in2 o in4, pin assegnato a ena o enb)
	: in1(in1), in2(in2), pwm(pwm), offset(offset), stby(stby)  // valori dichiarati piu sotto come privati che sono poi utilizzati all interno della classe MyMotor
	{};
	
TB6612Motor()::~TB6612Motor()
	{ 
		Stop();
	};	

void	TB6612Motor::Initialize();     
	{
		pinMode(in1,OUTPUT);
		pinMode(in2,OUTPUT);
		pinMode(pwm,OUTPUT);
		pinMode(stby,OUTPUT);
		digitalWrite(in1, LOW);	//aggiunti come da TheDIYGuy999 per mettersi in mode Stop con gli output a OFF
		digitalWrite(in2, LOW);	//aggiunti come da TheDIYGuy999 per mettersi in mode Stop con gli output a OFF
		digitalWrite(pwm, HIGH);	//aggiunti come da TheDIYGuy999 per mettersi in mode Stop con gli output a OFF
	}


void	TB6612Motor::Forward(int speed); 
	{
		digitalWrite(in1,LOW);
		digitalWrite(in2,HIGH);
		analogWrite(pwm,speed);
		digitalWrite(stby,HIGH);
	#ifdef DEBUG_TB
		Serial.println("go newforward!");
	#endif
	}

void	TB6612Motor::Backward(int speed); 
	{
		digitalWrite(in1,HIGH);
		digitalWrite(in2,LOW);
		analogWrite(pwm,speed);
		digitalWrite(stby,HIGH);
	#ifdef DEBUG_TB
		Serial.println("go newbackward!");
	#endif
	}

void	TB6612Motor::Stop();
	{
		digitalWrite(in1,LOW);
		digitalWrite(in2,LOW);
		analogWrite(pwm,LOW);
		digitalWrite(stby,LOW);  //    L		L		H		OFF		OFF		Stop
	#ifdef DEBUG_TB
		Serial.println("newStop!");
	#endif
	}
	


