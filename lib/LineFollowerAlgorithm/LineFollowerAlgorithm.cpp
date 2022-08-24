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

void LineFollowerAlgorithm::calibrateBackground()
{
  int backGroundReadArr[global::numberOfSensor];
  for(int i = 0; i < global::numberOfSensor; i++)
  {
    backGroundReadArr[i] = 0;
  }
  // Lendo a média do background
  //Repete por i vezes
  int repeatNumber = 50;
  for(int i = 0; i < repeatNumber; i++)
  {
    for(int j = 0; j < global::numberOfSensor; j++)
    {
      backGroundReadArr[j] += sensorArr[j].readValueAnalog();
    }
    delay(10);
  }

  for(int i = 0; i < global::numberOfSensor; i++)
  {
    backGroundReadArr[i] = backGroundReadArr[i] / repeatNumber;
    sensorArr[i].backgrounAnalogValue = backGroundReadArr[i];
  }

}

void LineFollowerAlgorithm::calibrateSensors()
{
  calibrateBackground();
    
  


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