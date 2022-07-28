#ifndef GENERICPID_H
#define GENERICPID_H

#include "Arduino.h"


class Pid
{
public:
	//Construtor do pid, devem ser passado as constantes
	Pid();
	Pid(double k_p, double k_i, double k_d);

	void setValues(Pid pidValues);

	//Funcao que realiza a computacao do pid (ValorDeObjetivo - ValorAtual)
	double calculate(double error);

	//Constantes, cuidado ao acessar elas para nao modificar sem querer
	//	i normalmente eh um valor bem pequeno i = 0.000001, nunca coloque i = 0
	double p; double i; double d;

	

private:
	double cumulativeError = 0;
	double lastError = 0;
};


#endif