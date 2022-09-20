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
    const int numberOfSensor = 10;
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

    

    Pid pidLow;
    Pid pidHigh;
    

    float lowGain = 0.7;
    float highGain = 1;
    float motorLimiter = 1; //Limite maximo do motor

private:
    float calculateSensValue();
    void setPidValues(Pid pidValues);

    //Calibra os sensores
    void calibrateSensors();


    int numberOfSensors = global::numberOfSensor;
    
    // Qual o objetivo, nesse caso por a linha entre o sensor S4 e S5
    float readingGoal = 4.5;
    float gain;
    Pid pidLeft;
    Pid pidRight;

    float sensorValue;



	bool lineIsBlack = true;

    

    bool outOfLine = true;


};




#endif