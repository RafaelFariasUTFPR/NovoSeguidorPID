#include "GenericPid.h"



Pid::Pid()
{
	p = 0.01;
	i = 0.0001;
	d = 0;

}

Pid::Pid(double k_p, double k_i, double k_d)
{
	p = k_p;
	i = k_i;
	d = k_d;

}

void Pid::setValues(Pid pidValues)
{
	p = pidValues.p;
	i = pidValues.i;
	d = pidValues.d;
}




//Fun��o que realiza a computa��o do pid (ValorDeObjetivo - ValorAtual)
/*
	Para calcular basta passar qual o erro, ou seja (ValorDeObjetivo - ValorAtual)

	

	Por exemplo, digamos que tenho 3 sensores
	
	S1	S2	S3

	meu objetivo � que a linha esteja sobre o S2, ou seja, meu ValorDeObjetivo = 2

	caso a linha esteja sobre o S3 o meu erro ser� de (2 - 3 = -1), ou seja preciso passar -1 como meu erro
*/
double Pid::calculate(double error)
{
	cumulativeError += error;

	//Resetando o erro acumulado ao alcan�ar o objetivo
	if (error == 0)
		cumulativeError = 0;

	//Limitando a integral
	if (cumulativeError > 50 / i)
		error = 50 / i;


	//Calculando os valores do PID
	double proportional = error * p;
	double integral = cumulativeError * i;
	double derivative = (error - lastError) * d;


	lastError = error;


	//Resultado da computa��o
	return proportional + integral + derivative;
}