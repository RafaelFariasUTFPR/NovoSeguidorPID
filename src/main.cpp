#include <Arduino.h>


#include "SensorLib.h"
#include "Motorlib.h"
#include "GenericPid.h"
#include "LineFollowerAlgorithm.h"
#include "RemoteSetup.h"


//  ###### Motor Setup ######
/*
#define PWMA 16
#define AIN2 4
#define AIN1 0
#define STBY 2
#define BIN1 15
#define BIN2 18
#define PWMB 12


*/

#define PWMA 23
#define AIN2 22
#define AIN1 21
#define STBY 19
#define BIN1 18
#define BIN2 5
#define PWMB 17


#define PWM_Left 0
#define PWM_Right 12
#define PWM_Res 8
#define PWM_Freq 10000
//  ###### ########## ######


//  ###### Sensor Setup ######
/*
#define S0 14
#define S1 27
#define S2 26
#define S3 25 
#define S4 33 //meio
#define S5 32 //meio
#define S6 35
#define S7 34
#define S8 39
#define S9 36

*/
/*
            S3  S4  S5  S6
    S1  S2                  S7  S8
S0                                  S9

*/


#define S2 39
#define S1 36
#define S3 34
#define S4 35
#define S5 32 //meio
#define S6 33
#define S7 25
#define S8 26
#define S9 27
#define S10 14
#define S11 13



#define SR 4
#define LED1_PIN 12



//  ###### ########## ######


Tb6612fng motorController(PWMA, AIN2, AIN1, STBY, BIN2, BIN1,
  PWMB, PWM_Left, PWM_Right, PWM_Res, PWM_Freq);

Pid pidLow(0.3, 0.0000001, 0.655);
Pid pidHigh(0.27, 0.0000001, 0.32); // velocidade
//Pid pidHigh(0.160, 0.0000001, 0.50); // curva 90°


//Criando o seguidor em si, e passando os valores das constantes
LineFollowerAlgorithm lineFollower(pidLow, pidHigh, motorController);

Remote remote(lineFollower);


//Adiciona os sensores ao algoritimo
void addSensors()
{
  lineFollower.addSensor(sensorTcrt5000(S2, 2082), 0);
  lineFollower.addSensor(sensorTcrt5000(S1, 1702), 1);
  lineFollower.addSensor(sensorTcrt5000(S3, 1671), 2);
  lineFollower.addSensor(sensorTcrt5000(S4, 1707), 3);
  lineFollower.addSensor(sensorTcrt5000(S5, 2094), 4);
  lineFollower.addSensor(sensorTcrt5000(S11, 2095), 5);
  lineFollower.addSensor(sensorTcrt5000(S6, 2097), 6);
  lineFollower.addSensor(sensorTcrt5000(S10, 2095), 7);
  lineFollower.addSensor(sensorTcrt5000(S9, 2084), 8);
  lineFollower.addSensor(sensorTcrt5000(S7, 2084), 9);
  lineFollower.addSensor(sensorTcrt5000(S8, 2084), 10);
  lineFollower.rightSensorPin = SR;
  lineFollower.ledPin = LED1_PIN;
}


void setup() 
{
  Serial.begin(115200);  
  addSensors();
  motorController.motorSetup();
  
  remote.beginBluetooth();

  //Calibração
  lineFollower.start();

  setCpuFrequencyMhz(240);
  Serial.println(getCpuFrequencyMhz());

}


void loop() 
{
  remote.process();

  lineFollower.process();

  //Serial.println(digitalRead(SR));
  //lineFollower.printAllSensors();
  //Serial.print(lineFollower.isRunning);
  //delay(10);
}

