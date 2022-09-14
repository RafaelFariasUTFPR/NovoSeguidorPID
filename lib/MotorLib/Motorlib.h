#ifndef MOTORLIB_H
#define MOTORLIB_H



#include "Arduino.h"



class Tb6612fng
{
public:
	Tb6612fng();
	Tb6612fng(
		int _PWMA, int _AIN2, int _AIN1, int _STBY, int _BIN2,
		int _BIN1, int _PWMB, int _PWM_Left, int _PWM_Right,
		int _PWM_Res, int _PWM_Freq);
	
	void motorTest();
	void motorSetup();
	void driveMotor(float leftMotor, float rightMotor);

	void setLeftMotorForward();
	void setRightMotorForward();
	void setLeftMotorBackward();
	void setRightMotorBackward();

	void restartMotor();



	int PWMA, AIN2, AIN1, STBY, BIN2, BIN1, PWMB, PWM_Left, 
	PWM_Right, PWM_Res, PWM_Freq;

};


#endif //MOTORLIB_H