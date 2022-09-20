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
#define PWMB 12



#define PWM_Left 1
#define PWM_Right 8
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
// "." (46) - Modo Baixo Ganho
// "/" (47) - Modo Alto Ganho

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
Pid pidLow(0.25, 0.000001, 0.72);
Pid pidHigh(0.25, 0.000001, 0.72);

LineFollowerAlgorithm lineFollower(pidLow, pidHigh, motorController);

bool lowGainMode = false;


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
  if(lineFollower.isRunning)
    return;
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
  if(lineFollower.isRunning)
    return;
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
  if(lowGainMode)
  {
    bluetoothModule.print(lineFollower.pidLow.p, 3);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower.pidLow.i * 1000, 3);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower.pidLow.d, 3);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower.lowGain, 3);
    bluetoothModule.println();

    Serial.print("LOW PID: ");
    Serial.print(lineFollower.pidLow.p,3);
    Serial.print(",");
    Serial.print(lineFollower.pidLow.i,7);
    Serial.print(",");
    Serial.print(lineFollower.pidLow.d,3);
    Serial.print(",");
    Serial.print(lineFollower.lowGain,3);
    Serial.println();
    return;
  }
  bluetoothModule.print(lineFollower.pidHigh.p, 3);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.pidHigh.i * 1000, 3);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.pidHigh.d, 3);
  bluetoothModule.print(",");
  bluetoothModule.print(lineFollower.highGain, 3);
  bluetoothModule.println();

  Serial.print("HIGH PID: ");
  Serial.print(lineFollower.pidHigh.p,3);
  Serial.print(",");
  Serial.print(lineFollower.pidHigh.i,7);
  Serial.print(",");
  Serial.print(lineFollower.pidHigh.d,3);
  Serial.print(",");
  Serial.print(lineFollower.highGain,3);
  Serial.println();



}

void setup() 
{
  Serial.begin(115200);  
  
  //Adiciona os sensores ao seguidor
  addSensors();
  motorController.motorSetup();
  //motorController.driveMotor(0.8,0.8);
  //Calibração
  bluetoothModule.begin("SEGUIDOR_ESP32");
  Serial.println("Bluetooth Device is Ready to Pair");

  lineFollower.start();
  //lineFollower.calibrateSensors();


}

void changePID(float variation, char c)
{
  switch (c)
  {
  case 'p':
    if(lowGainMode)
    {
      lineFollower.pidLow.p += variation;
      //lineFollower.setPidLowValue();
    }
    else
    {
      lineFollower.pidHigh.p += variation;
      //lineFollower.setPidHighValue();
    }
    break;
  case 'i':
    if(lowGainMode)
    {
      lineFollower.pidLow.i += variation/1000;
      //lineFollower.setPidLowValue();
    }
    else
    {
      lineFollower.pidHigh.i += variation/1000;
      //lineFollower.setPidHighValue();
    }
    break;
  case 'd':
    if(lowGainMode)
    {
      lineFollower.pidLow.d += variation;
      //lineFollower.setPidLowValue();
    }
    else
    {
      lineFollower.pidHigh.d += variation;
      //lineFollower.setPidHighValue();
    }
    break;
  case 'm':
    if(lowGainMode)
      lineFollower.lowGain += variation;
    else
      lineFollower.highGain += variation;
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
    Serial.print("Received:"); 
    Serial.println(incoming);

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
      case 46: // "." (46) - Modo Baixo Ganho
        lowGainMode = true;
        //lineFollower.setPidLowValue();
        break;
      case 47: // "/" (47) - Modo Alto Ganho
        lowGainMode = false;
        //lineFollower.setPidHighValue();
        break;
      case 97: // "a" (97) - -0.001
        changePID(-0.001, mode);
        sendPID();
        break;
      case 98: // "b" (98) - -0.01
        changePID(-0.01, mode);
        sendPID();
        break;
      case 99: // "c" (99) - -0.1
        changePID(-0.1, mode);
        sendPID();
        break;
      case 100: // "d" (100) - 0.1
        changePID(0.1, mode);
        sendPID();
        break;
      case 101: // "e" (101) - 0.01
        changePID(0.01, mode);
        sendPID();
        break;
      case 102: // "f" (102) - 0.001
        changePID(0.001, mode);
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

      case 114: // "r" (114) - restartMotor
        Serial.println("Restart");
        lineFollower.motorController->restartMotor();

        break;
      }
    }
   

  }

  lineFollower.process();

  //delay(10);
  //lineFollower.printAllSensors();

}

