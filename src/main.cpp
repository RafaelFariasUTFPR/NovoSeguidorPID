#include <Arduino.h>


#include "SensorLib.h"
#include "Motorlib.h"
#include "GenericPid.h"
#include "LineFollowerAlgorithm.h"


//  ###### Motor Setup ######
#define PWMA 4
#define AIN2 18
#define AIN1 17
#define STBY 19
#define BIN2 16
#define BIN1 15
#define PWMB 2

#define PWM_Left 0
#define PWM_Right 1
#define PWM_Res 8
#define PWM_Freq 1000
//  ###### ########## ######


//  ###### Sensor Setup ######
#define S0 27
#define S1 26
#define S2 25
#define S3 33 
#define S4 32
#define S5 35
#define S6 34
#define S7 14
#define S8 39
#define S9 36

/*
            S3  S4  S5  S6
    S1  S2                  S7  S8
S0                                  S9

*/

//  ###### ########## ######



Tb6612fng motorController(PWMA, AIN2, AIN1, STBY, BIN2, BIN1,
  PWMB, PWM_Left, PWM_Right, PWM_Res, PWM_Freq);

//Criando o seguidor em si, e passando os valores das constantes
LineFollowerAlgorithm lineFollower(Pid(0.1, 0.000001, 0), motorController);


//Adiciona os sensores ao algoritimo
void addSensors()
{
  lineFollower.addSensor(sensorTcrt5000(S0));
  lineFollower.addSensor(sensorTcrt5000(S1));
  lineFollower.addSensor(sensorTcrt5000(S2));
  lineFollower.addSensor(sensorTcrt5000(S3));
  lineFollower.addSensor(sensorTcrt5000(S4));
  lineFollower.addSensor(sensorTcrt5000(S5));
  lineFollower.addSensor(sensorTcrt5000(S6));
  lineFollower.addSensor(sensorTcrt5000(S7));
  lineFollower.addSensor(sensorTcrt5000(S8));
  lineFollower.addSensor(sensorTcrt5000(S9));
}


void setup() 
{
  //Adiciona os sensores ao seguidor
  addSensors();

  //Calibração
  lineFollower.calibrateSensors();


  Serial.begin(115200);  
}

void loop() 
{
  // lineFollower.motorController -> motorTest();

  //Aqui estou testando o commit
  lineFollower.run();
}