#ifndef SENSORLIB_H
#define SENSORLIB_H

#include "Arduino.h"

class sensorTcrt5000
{
public:
	sensorTcrt5000();
	sensorTcrt5000(int pino);
	sensorTcrt5000(int pino, int _midPoint);
	
	//Quando não ha reflexçao o valor é 0
	int readValueAnalog();

	int readValue();

	void calibrate();
	void getBackgroundColor();
	


	int pin; //Qual pino o sensor est� ligado

	bool lineIsBlack = true; // Corrigir

	int lowValue = 0x3f3f3f3f, highValue = 0;

	int midPoint = 110; //Offset antes de retornar true ( retira o ru�do ) 


private:
	bool calibrated = false;
};


#endif
