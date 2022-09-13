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
    for(int i = 0; i < numberOfSensors; i++)
      sensorArr[i].calibrate();
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

  leftMotor *= motorMultiplier;
  rightMotor *= motorMultiplier;
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

void LineFollowerAlgorithm::printAllSensors()
{
  for(int i = 0; i < global::numberOfSensor; i++)
  {
    Serial.print(sensorArr[i].readValue());
    Serial.print(",");

  }
  Serial.print("\n");
}
void LineFollowerAlgorithm::printAllSensorsAnalog()
{
  for(int i = 0; i < global::numberOfSensor; i++)
  {
    Serial.print(sensorArr[i].readValueAnalog());
    Serial.print(",");

  }
  Serial.print("\n");
}

void LineFollowerAlgorithm::testMotors()
{
  motorController ->motorTest();
}

void LineFollowerAlgorithm::process()
{
  if(isCallibrating)
    calibrateSensors();

  if(isRunning)
    run();
  
  if(!isRunning)
    motorController -> driveMotor(0, 0);

}