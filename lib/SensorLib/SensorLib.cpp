#include "SensorLib.h"


sensorTcrt5000::sensorTcrt5000()
{
	pin = 0;
}

sensorTcrt5000::sensorTcrt5000(int pino)
{
	pin = pino;
}

sensorTcrt5000::sensorTcrt5000(int pino, int _midPoint)
{
	pin = pino;
	midPoint = _midPoint;
}


int sensorTcrt5000::readValueAnalog()
{
	return analogRead(pin);
}


int sensorTcrt5000::readValue()
{
	if (analogRead(pin) > midPoint)
		return 1;
	return 0;
}


void sensorTcrt5000::calibrate()
{

	if(readValueAnalog() < lowValue)
		lowValue = readValueAnalog();

	if(readValueAnalog() > highValue)
		highValue = readValueAnalog();
	
	midPoint = (lowValue + highValue) / 2; 
	
	Serial.print(midPoint);
	Serial.print(",");
}
