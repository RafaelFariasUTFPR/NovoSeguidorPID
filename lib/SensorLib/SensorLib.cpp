#include "SensorLib.h"


sensorTcrt5000::sensorTcrt5000(int pino)
{
	pin = pino;
}

sensorTcrt5000::sensorTcrt5000(int pino, int offsetRuido)
{
	pin = pino;
	threshold = offsetRuido;
}


int sensorTcrt5000::readValueAnalog()
{
	return analogRead(pin);
}


int sensorTcrt5000::readValue()
{
	if (analogRead(pin) > threshold)
		return 1;
	return 0;
}


bool sensorTcrt5000::calibrate()
{
	//Talvez essa função esteja ao contrário, ou seja talvez seja necessario
	//inverter as adições e o loop, para que ele diminua ao invez de crescer

	if(calibrated)
		return true;
	
	for(int i = 0; i <= 4095; i += 50)
	{
		//Altera o valor do threshold, alterando assim a função "readValue()"
		threshold = i;
		if(readValue())
		{
			threshold += sensorReadOffset;
			calibrated = true;
			return true;
		}
	}

	//Retorna verdadeiro caso a calibração tenha funcionado
	return false;
}
