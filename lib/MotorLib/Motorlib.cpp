#include "Motorlib.h"


Tb6612fng::Tb6612fng()
{
	PWMA = 4;
	AIN2 = 18;
	AIN1 = 17;
	STBY = 19;
	BIN2 = 16;
	BIN1 = 15;
	PWMB = 2;
	PWM_Left = 0;
	PWM_Right = 1;
	PWM_Res = 8;
	PWM_Freq = 1000;

	motorSetup();
}

Tb6612fng::Tb6612fng(
	int _PWMA, int _AIN2, int _AIN1, int _STBY, int _BIN2,
	int _BIN1, int _PWMB, int _PWM_Left, int _PWM_Right,
	int _PWM_Res, int _PWM_Freq)
{
	PWMA = _PWMA;
	AIN2 = _AIN2;
	AIN1 = _AIN1;
	STBY = _STBY;
	BIN2 = _BIN2;
	BIN1 = _BIN1;
	PWMB = _PWMB;
	PWM_Left = _PWM_Left;
	PWM_Right = _PWM_Right;
	PWM_Res = _PWM_Res;
	PWM_Freq = _PWM_Freq;

	motorSetup();
}

void Tb6612fng::motorTest()
{
	ledcWrite(PWM_Left, 100);
	ledcWrite(PWM_Right, 100);
	delay(2000);
	ledcWrite(PWM_Left, 255);
	ledcWrite(PWM_Right, 255);
	delay(1000);
	ledcWrite(PWM_Left, 500);
	ledcWrite(PWM_Right, 500);
	delay(1000);
	ledcWrite(PWM_Left, 0);
	ledcWrite(PWM_Right, 0);
}

void Tb6612fng::motorSetup()
{
	pinMode(PWMA, OUTPUT);
	pinMode(AIN2, OUTPUT);
	pinMode(AIN1, OUTPUT);
	pinMode(STBY, OUTPUT);
	pinMode(BIN2, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);
	pinMode(PWMB, OUTPUT);

	digitalWrite(STBY, HIGH);

	ledcSetup(PWM_Left, PWM_Freq, PWM_Res);
	ledcAttachPin(PWMA, PWM_Left);

	ledcSetup(PWM_Right, PWM_Freq, PWM_Res);
	ledcAttachPin(PWMB, PWM_Right);

	ledcWrite(PWM_Left, 0);
	ledcWrite(PWM_Right, 0);
}

void Tb6612fng::setLeftMotorForward()
{
	digitalWrite(AIN1, HIGH);
	digitalWrite(AIN2, LOW);

}

void Tb6612fng::setRightMotorForward()
{
	digitalWrite(BIN1, HIGH);
	digitalWrite(BIN2, LOW);

}

void Tb6612fng::setLeftMotorBackward()
{
	digitalWrite(AIN1, LOW);
	digitalWrite(AIN2, HIGH);

}

void Tb6612fng::setRightMotorBackward()
{
	digitalWrite(BIN1, LOW);
	digitalWrite(BIN2, HIGH);

}




void Tb6612fng::driveMotor(float leftMotor, float rightMotor)
{
	int leftVal = int (leftMotor * 255);
	int rightVal = int (rightMotor * 255);

	if (leftVal > 0)
		setLeftMotorForward();
	else
		setLeftMotorBackward();

	if (rightVal > 0)
		setRightMotorForward();
	else
		setRightMotorBackward();

	ledcWrite(PWM_Left, abs(leftVal));
	ledcWrite(PWM_Right, abs(rightVal));

}
