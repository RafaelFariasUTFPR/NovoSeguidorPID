#ifndef SENSORLIB_H
#define SENSORLIB_H

#include "Arduino.h"

class sensorTcrt5000
{
public:
	sensorTcrt5000(int pino);
	sensorTcrt5000(int pino, int offsetRuido);
	
	//Quando não ha reflexçao o valor é 0
	int readValueAnalog();

	int readValue();

	bool calibrate();

	


	int pin; //Qual pino o sensor est� ligado
	

	int threshold = 1000; //Offset antes de retornar true ( retira o ru�do ) 

	int sensorReadOffset = 100;

protected:
	bool calibrated = false;
};


#endif
