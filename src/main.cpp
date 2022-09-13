#include <Arduino.h>

#include "BluetoothSerial.h"


#include "SensorLib.h"
#include "Motorlib.h"
#include "GenericPid.h"
#include "LineFollowerAlgorithm.h"


// Corrigir os pinos do motor
//  ###### Motor Setup ######

#define PWMA 16
#define AIN2 4
#define AIN1 0
#define STBY 2
#define BIN1 15
#define BIN2 18
#define PWMB 5
/*
#define PWMA 15
#define AIN2 15
#define AIN1 15
#define STBY 15
#define BIN1 15
#define BIN2 15
#define PWMB 15
*/


#define PWM_Left 0
#define PWM_Right 1
#define PWM_Res 8
#define PWM_Freq 5000
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


// "!" (33) - Transmitir leitura
// "#" (35) - Iniciar
// "%" (37) - Parar
// "'" (39) - Calibrar
// ")" (41) - Encerrar calibração
// "+" (43) - Atualizar média
// "1" (49) - Nivel 1
// "2" (50) - Nivel 2
// "3" (51) - Nivel 3

BluetoothSerial bluetoothModule;

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

void sendAnalogRead()
{
  bluetoothModule.print(lineFollower.sensorArr[0].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[1].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[2].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[3].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[4].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[5].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[6].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[7].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[8].readValueAnalog());
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[9].readValueAnalog());
  bluetoothModule.println();
}

void sendMidPoint()
{
  bluetoothModule.print(lineFollower.sensorArr[0].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[1].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[2].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[3].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[4].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[5].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[6].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[7].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[8].midPoint);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.sensorArr[9].midPoint);
  bluetoothModule.println();
}

void setup() 
{
  Serial.begin(115200);  
  Serial.println(1);
  
  //Adiciona os sensores ao seguidor
  addSensors();
  motorController.motorSetup();
  //motorController.driveMotor(0.8,0.8);
  //Calibração
  bluetoothModule.begin("SEGUIDOR_ESP32");
  Serial.println("Bluetooth Device is Ready to Pair");

  //lineFollower.calibrateSensors();


}

void loop() 
{

  if (bluetoothModule.available()) //Check if we receive anything from Bluetooth
  {
    int incoming = bluetoothModule.read(); //Read what we recevive
    // Serial.print("Received:"); 
    // Serial.println(incoming);
    switch (incoming)
    {
    case 33: // "!" (33) - Transmitir leitura
      sendAnalogRead();
      lineFollower.printAllSensorsAnalog();

      break;

    case 35: // "#" (35) - Iniciar
      Serial.println("Iniciar"); 
      lineFollower.isRunning = true;
      break;

    case 37: // "%" (37) - Parar
      Serial.println("Parar"); 
      lineFollower.isRunning = false;
      break;

    case 39: // "'" (39) - Calibrar
      Serial.println("Calibrar"); 
      lineFollower.isCallibrating = true;
      Serial.println(",");

      break;

    case 41: // ")" (41) - Encerrar calibração
      Serial.println("Encerrar calibracao"); 
      lineFollower.isCallibrating = false;
      break;

    case 43: // "+" (43) - Atualizar média
      Serial.println("Atualizar");
      sendMidPoint();
      break;

    case 49: // "1" (49) - Nivel 1
      Serial.println("Nivel 1");

      break;

    case 50: // "2" (50) - Nivel 2
      Serial.println("Nivel 2");

      break;

    case 51: // "3" (51) - Nivel 3
      Serial.println("Nivel 3");

      break;
      
    default:
      break;
    }
   

  }

  lineFollower.process();

  // lineFollower.motorController -> motorTest();

  //lineFollower.run();

  //Testando os sensores
  
  //Serial.println(1);

  //lineFollower.testMotors();

}

