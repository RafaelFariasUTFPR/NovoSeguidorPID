#ifndef LINEFOLLOWERALGORITHM_H  
#define LINEFOLLOWERALGORITHM_H


#include "Arduino.h"
#include "GenericPid.h"
#include "Motorlib.h"
#include "SensorLib.h"


namespace global
{
    const int numberOfSensor = 10;
}


class LineFollowerAlgorithm
{
public:
    LineFollowerAlgorithm();
    LineFollowerAlgorithm(Pid pidValues);
    LineFollowerAlgorithm(Pid pidValues, Tb6612fng& _motorController);




    //Adiciona sensores ao vetor de sensores
    void addSensor(sensorTcrt5000 sensor, int index);




    void setPidValues(Pid pidValues);
    void setReadingGoal(float _readingGoal);

    void run();

    void process();

    // ##### DEBUG #####
    void printAllSensors();
    void printAllSensorsAnalog();

    void testMotors();

    // ##### ##### #####
    

    sensorTcrt5000 sensorArr [global::numberOfSensor];
    Tb6612fng* motorController;

    //REMOVER O TRUE
    bool isRunning = false; // REMOVER
    // !!!!!!!!!!!!!!

    bool isCallibrating = false; // Se for true entrará no modo de calibração

    Pid pidLeft;
    Pid pidRight;

    float maxSpeed = 0.5;
    float motorLimiter = 1; //Limite maximo do motor

private:
    float calculateSensValue();

    //Calibra os sensores
    void calibrateSensors();

    int numberOfSensors = global::numberOfSensor;
    
    // Qual o objetivo, nesse caso por a linha entre o sensor S4 e S5
    float readingGoal = 4.5;

    float sensorValue;



	bool lineIsBlack = true;

    

    bool outOfLine = true;


};




#endif