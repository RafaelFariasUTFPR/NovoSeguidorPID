#include <Arduino.h>

#include "BluetoothSerial.h"


#include "SensorLib.h"
#include "Motorlib.h"
#include "GenericPid.h"
#include "LineFollowerAlgorithm.h"


//  ###### Motor Setup ######
#define PWMA 16
#define AIN2 4
#define AIN1 0
#define STBY 2
#define BIN1 15
#define BIN2 18
#define PWMB 5



#define PWM_Left 0
#define PWM_Right 2
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
// "-" (45) - Modo setup
// "1" (49) - Nivel 1
// "2" (50) - Nivel 2
// "3" (51) - Nivel 3



// " " " (34) - Modo P
// "$" (36) - Modo I
// "&" (38) - Modo D
// "(" (40) - Modo M
// "*" (42) - Atualizar PID
// "," (44) - Sair do setup
// "a" (97) - -0.001
// "b" (98) - -0.01
// "c" (99) - -0.1
// "d" (100) - +0.1
// "e" (101) - +0.01
// "f" (102) - +0.001


BluetoothSerial bluetoothModule;

Tb6612fng motorController(PWMA, AIN2, AIN1, STBY, BIN2, BIN1,
  PWMB, PWM_Left, PWM_Right, PWM_Res, PWM_Freq);

//Criando o seguidor em si, e passando os valores das constantes
LineFollowerAlgorithm lineFollower(Pid(0.1, 0.0002, 0), motorController);

//Adiciona os sensores ao algoritimo
void addSensors()
{
  lineFollower.addSensor(sensorTcrt5000(S0, 2077), 0);
  lineFollower.addSensor(sensorTcrt5000(S1, 2082), 1);
  lineFollower.addSensor(sensorTcrt5000(S2, 1702), 2);
  lineFollower.addSensor(sensorTcrt5000(S3, 1671), 3);
  lineFollower.addSensor(sensorTcrt5000(S4, 1707), 4);
  lineFollower.addSensor(sensorTcrt5000(S5, 2094), 5);
  lineFollower.addSensor(sensorTcrt5000(S6, 2095), 6);
  lineFollower.addSensor(sensorTcrt5000(S7, 2097), 7);
  lineFollower.addSensor(sensorTcrt5000(S8, 2095), 8);
  lineFollower.addSensor(sensorTcrt5000(S9, 2084), 9);
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

void sendPID()
{
  bluetoothModule.print(lineFollower.pidLeft.p);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.pidLeft.i);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.pidLeft.d);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.maxSpeed);
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

void chanchePID(float variation, char c)
{
  switch (c)
  {
  case 'p':
    lineFollower.pidLeft.p += variation;
    lineFollower.pidRight.p += variation;
    break;
  case 'i':
    lineFollower.pidLeft.i += variation;
    lineFollower.pidRight.i += variation;
    break;
  case 'd':
    lineFollower.pidLeft.d += variation;
    lineFollower.pidRight.d += variation;
    break;
  
  default:
    break;
  }
}

bool setupMode = false;

char mode = 'p';

void loop() 
{

  if (bluetoothModule.available()) //Check if we receive anything from Bluetooth
  {
    int incoming = bluetoothModule.read();
    // Serial.print("Received:"); 
    // Serial.println(incoming);

    if(setupMode)
    {
      switch (incoming)
      {
      case 34: // " " " (34) - Modo P
        mode = 'p';
        break;
      case 36: // "$" (36) - Modo I
        mode = 'i';
        break;
      case 38: // "&" (38) - Modo D
        mode = 'd';
        break;
      case 40: // "(" (40) - Modo M
        mode = 'm';
        break;
      case 42: // "*" (42) - Atualizar PID
        sendPID();
        break;
      case 44: // "," (44) - Sair do setup
        setupMode = false;
        break;
      case 97: // "a" (97) - -0.001
        chanchePID(-0.001, mode);
        sendPID();
        break;
      case 98: // "b" (98) - -0.01
        chanchePID(-0.01, mode);
        sendPID();
        break;
      case 99: // "c" (99) - -0.1
        chanchePID(-0.1, mode);
        sendPID();
        break;
      case 100: // "d" (100) - 0.1
        chanchePID(0.1, mode);
        sendPID();
        break;
      case 101: // "e" (101) - 0.01
        chanchePID(0.01, mode);
        sendPID();
        break;
      case 102: // "f" (102) - 0.001
        chanchePID(0.001, mode);
        sendPID();
        break;

      }
    }
    else
    {
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
      
      case 45: // "-" (45) - Modo setup
        Serial.println("Setup");
        setupMode = true;
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
      }
    }
   

  }

  lineFollower.process();


  //lineFollower.printAllSensors();

}

