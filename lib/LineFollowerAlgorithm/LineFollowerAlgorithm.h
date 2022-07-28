#ifndef LINEFOLLOWERALGORITHM_H  
#define LINEFOLLOWERALGORITHM_H


#include "Arduino.h"
#include "GenericPid.h"
#include "Motorlib.h"
#include "SensorLib.h"
#include "Vector.h"

class LineFollowerAlgorithm
{
public:
    LineFollowerAlgorithm();
    LineFollowerAlgorithm(Pid pidValues);
    LineFollowerAlgorithm(Pid pidValues, Tb6612fng& _motorController);



    //Adiciona sensores ao vetor de sensores
    void addSensor(sensorTcrt5000 sensor);

    //Calibra os sensores
    void calibrateSensors();


    void setPidValues(Pid pidValues);
    void setReadingGoal(float _readingGoal);

    void run();


    Vector<sensorTcrt5000> sensorVector;
    Tb6612fng* motorController;


protected:
    float calculateSensValue();

    unsigned int numberOfSensors;
    
    // Qual o objetivo, nesse caso por a linha entre o sensor S4 e S5
    float readingGoal = 4.5;

    float sensorValue;
    Pid pidLeft;
    Pid pidRight;

    // Controla a velocidade dos motores -1 a 1
    float leftMotor;
    float rightMotor;

    float maxSpeed = 0.8;
    


};




#endif