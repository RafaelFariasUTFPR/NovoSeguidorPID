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

  float leftMotor = maxSpeed;
  float rightMotor = maxSpeed;
  leftMotor += pidLeft.calculate(readingGoal - calculateSensValue());
  rightMotor += pidRight.calculate(calculateSensValue() - readingGoal);
  






  if(leftMotor > motorLimiter)
    leftMotor = motorLimiter;
  if(leftMotor < -motorLimiter)
    leftMotor = -motorLimiter;
  if(rightMotor > motorLimiter)
    rightMotor = motorLimiter;
  if(rightMotor < -motorLimiter)
    rightMotor = -motorLimiter;
  

  //Serial.print(leftMotor);
  //Serial.print(", ");
  //Serial.println(rightMotor);
  


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
      float errorMultiplier;
      switch (i)
      {
      case 2:
        errorMultiplier = 2.8;
        break;
      case 3:
        errorMultiplier = 3.5;
        break;
      case 4:
        errorMultiplier = 4.3;
        break;
      case 5:
        errorMultiplier = 4.7;
        break;
      case 6:
        errorMultiplier = 5.5;
        break;    
      case 7:
        errorMultiplier = 6.2;
        break;     
      default:
        errorMultiplier = float(i);
        break;
      }

      result += errorMultiplier;
      numOfTrueSensors++;
    }
  }

  if(numOfTrueSensors == 0)
  {
    outOfLine = true;
    return readingGoal;
  }
  //Serial.println(result /numOfTrueSensors);
  outOfLine = false;
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
  {
    motorController -> driveMotor(0, 0);

  }

}