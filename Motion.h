/* Motion.h*/
/* Class to manage motion using 2 motors*/

#ifndef MOTION_H
#define MOTION_H

#include TB6612FNG_DC_Motor_motor_driver.h

class Motion 
{
	public:
	Motion ( );
	~Motion( );
	
	/* initialization for 2 motors 
	give pin for motor a and b plugged on TB6612FNG */
	Init( int ina1, int ina2, int pwma, int offseta, int stbya, int inb1, int inb2, int pwmb, int offsetb, int stbyb ) ;

	
	private :
	
	TB6612Motor motor1, motor2;
	
	
	
	
};





#endif