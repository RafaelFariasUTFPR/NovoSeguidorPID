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
  lineFollower.addSensor(sensorTcrt5000(S0), 0);
  lineFollower.addSensor(sensorTcrt5000(S1), 1);
  lineFollower.addSensor(sensorTcrt5000(S2), 2);
  lineFollower.addSensor(sensorTcrt5000(S3), 3);
  lineFollower.addSensor(sensorTcrt5000(S4), 4);
  lineFollower.addSensor(sensorTcrt5000(S5), 5);
  lineFollower.addSensor(sensorTcrt5000(S6), 6);
  lineFollower.addSensor(sensorTcrt5000(S7), 7);
  lineFollower.addSensor(sensorTcrt5000(S8), 8);
  lineFollower.addSensor(sensorTcrt5000(S9), 9);
}


void setup() 
{
  Serial.begin(115200);  
  
  
  //Adiciona os sensores ao seguidor
  addSensors();

  //Calibração
  //lineFollower.calibrateSensors();
  //Asd

}

void loop() 
{
  // lineFollower.motorController -> motorTest();

  //lineFollower.run();

  //Testando os sensores
  
  
  for(int i = 0; i < global::numberOfSensor; i++)
  {
    Serial.print(lineFollower.sensorArr[i].readValue());
    Serial.print(",");

  }
  Serial.print("\n");
  
}

