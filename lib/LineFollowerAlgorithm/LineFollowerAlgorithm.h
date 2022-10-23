#ifndef LINEFOLLOWERALGORITHM_H  
#define LINEFOLLOWERALGORITHM_H

#include <EEPROM.h>

#include "Arduino.h"
#include "GenericPid.h"
#include "Motorlib.h"
#include "SensorLib.h"

//#define EEPROM_SIZE 10


namespace global
{
    const int numberOfSensor = 11;
}

bool almostEqual(float number, float objective, float precision);

class LineFollowerAlgorithm
{
public:
    LineFollowerAlgorithm();
    LineFollowerAlgorithm(Pid _pidLow, Pid _pidHigh);
    LineFollowerAlgorithm(Pid _pidLow, Pid _pidHigh, Tb6612fng& _motorController);




    //Adiciona sensores ao vetor de sensores
    void addSensor(sensorTcrt5000 sensor, int index);



    void setPidLowValue(){setPidValues(pidLow); gain = lowGain;}
    void setPidHighValue(){setPidValues(pidHigh); gain = highGain;}
    void setPidOffLineValue(){setPidValues(pidLow); gain = offLineGain;}


    void setReadingGoal(float _readingGoal);

    void start();
    void run();

    void process();

    void saveCalibration();
    void loadCalibration();

    // ##### DEBUG #####
    void printAllSensors();
    void printAllSensorsAnalog();

    void testMotors();

    // ##### ##### #####
    
    void checkLineColor();


    sensorTcrt5000 sensorArr [global::numberOfSensor];
    Tb6612fng* motorController;

    //REMOVER O TRUE
    bool isRunning = false; // REMOVER
    // !!!!!!!!!!!!!!

    bool isCallibrating = false; // Se for true entrará no modo de calibração

    int rightSensorPin;
    int ledPin;

    Pid pidLow;
    Pid pidHigh;
    

    float lowGain = 0.34;
    //float highGain = 0.77; // Velocidade
    float highGain = 0.33; // Curva 90°
    float offLineGain = 0.3;
    
    int cyclesOnLine = 20;
    float motorLimiter = 0.99; //Limite maximo do motor

private:
    float calculateSensValue();
    void setPidValues(Pid pidValues);

    //Calibra os sensores
    void calibrateSensors();

    void setGain(float _result);


    int numberOfSensors = global::numberOfSensor;
    
    // Qual o objetivo, nesse caso por a linha entre o sensor S4 e S5
    float readingGoal = 5.5;
    float gain;
    float maxCenterOffset = 1;
    float outOfLineReading = 3;

    Pid pidLeft;
    Pid pidRight;

    float sensorValue;

    float lastReading = 0;

	bool lineIsBlack = true;
    int onLineTime;

    
    bool outOfTrack = false;
    bool outOfLine = false;


    unsigned long lastCrossing = 0;

};




#endif