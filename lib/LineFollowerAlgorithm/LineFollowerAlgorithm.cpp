#include "LineFollowerAlgorithm.h"


LineFollowerAlgorithm::LineFollowerAlgorithm()
{
    
}

LineFollowerAlgorithm::LineFollowerAlgorithm(Pid pidValues)
{
    pidLeft.setValues(pidValues);
    pidRight.setValues(pidValues);   
}

LineFollowerAlgorithm::LineFollowerAlgorithm(Pid pidValues, Tb6612fng& _motorController)
{
    pidLeft.setValues(pidValues);
    pidRight.setValues(pidValues);   
    motorController = &_motorController;
}


void LineFollowerAlgorithm::addSensor(sensorTcrt5000 sensor, int index)
{
    sensorArr[index] = sensor;
}

void LineFollowerAlgorithm::calibrateSensors()
{
    bool repeat = true;
    while(repeat)
    {
        repeat = false;
        for(int i = 0; i < numberOfSensors; i++)
        {
            if(!sensorArr[i].calibrate())
                repeat = true;
        }
    }

}


void LineFollowerAlgorithm::setPidValues(Pid pidValues)
{
  pidLeft.setValues(pidValues);
  pidRight.setValues(pidValues);
}


void LineFollowerAlgorithm::setReadingGoal(float _readingGoal)
{
  readingGoal = _readingGoal;
}


void LineFollowerAlgorithm::run()
{
  calculateSensValue();

  leftMotor += pidLeft.calculate(sensorValue - readingGoal);
  rightMotor += pidRight.calculate(readingGoal - sensorValue);


  motorController -> driveMotor(leftMotor, rightMotor);

}


float LineFollowerAlgorithm::calculateSensValue()
{
  float result = 0;

  //Numero de sensores em cima da linha
  int numOfTrueSensors = 0;
  
  for(int i = 0; i < numberOfSensors; i++)
  {
    if(sensorArr[i].readValue())
    {
      result += float(i);
      numOfTrueSensors++;
    }
  }

  //Retorna a média dos sensores ativos
  return result / numOfTrueSensors;
}