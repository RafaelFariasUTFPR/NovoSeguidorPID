#include "LineFollowerAlgorithm.h"


bool almostEqual(float number, float objective, float precision)
{
  if(number < objective + precision && number > objective - precision)
    return true;
  
  return false;
}


LineFollowerAlgorithm::LineFollowerAlgorithm()
{

}

LineFollowerAlgorithm::LineFollowerAlgorithm(Pid _pidLow, Pid _pidHigh)
{
  pidLow = _pidLow;
  pidHigh = _pidHigh;
  setPidLowValue();
 
}

LineFollowerAlgorithm::LineFollowerAlgorithm(Pid _pidLow, Pid _pidHigh, Tb6612fng& _motorController)
{
  pidLow = _pidLow;
  pidHigh = _pidHigh;
  setPidLowValue(); 
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

void LineFollowerAlgorithm::start()
{
    //loadCalibration();
}

void LineFollowerAlgorithm::run()
{
  checkLineColor();
  calculateSensValue();

  float leftMotor = gain;
  float rightMotor = gain;
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

void LineFollowerAlgorithm::checkLineColor()
{
  if(sensorArr[0].readValueAnalog() > sensorArr[0].midPoint && sensorArr[numberOfSensors-1].readValueAnalog() > sensorArr[numberOfSensors-1].midPoint)
  {
    for(int i = 0; i < numberOfSensors; i++)
      sensorArr[i].lineIsBlack = false;
    //Serial.println("White");
    return;
  }
  
  if(sensorArr[0].readValueAnalog() < sensorArr[0].midPoint && sensorArr[numberOfSensors-1].readValueAnalog() < sensorArr[numberOfSensors-1].midPoint)
  {
    for(int i = 0; i < numberOfSensors; i++)
      sensorArr[i].lineIsBlack = true;
    //Serial.println("Black");
    return;
  }

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
      case 1:
        errorMultiplier = 1;
        break;
      case 2:
        errorMultiplier = 1.8;
        break;
      case 3:
        errorMultiplier = 3.2;
        break;
      case 4:
        errorMultiplier = 4.1;
        break;
      case 5:
        errorMultiplier = 4.9;
        break;
      case 6:
        errorMultiplier = 5.8;
        break;    
      case 7:
        errorMultiplier = 7.2;
        break;   
      case 8:
        errorMultiplier = 8;
        break;   
        
      default:
        errorMultiplier = float(i);
        break;
      }

      result += errorMultiplier;
      if(almostEqual(result, readingGoal, 1))
      {
        pidLeft.zerarError();
        pidRight.zerarError();
      }
      numOfTrueSensors++;
    }
  }

  // Fazendo a volta se perder os sensores
  if(numOfTrueSensors == 0 || numOfTrueSensors == numberOfSensors)
  {
    outOfLine = true;
    // Caso a ultima leitura tenha sido mais ou menos no centro
    if(almostEqual(lastReading, readingGoal, maxCenterOffset))
    {
      return readingGoal;
    }
    // Caso a ultima leitura tenha sido para a esquerda
    else if(lastReading < readingGoal)
    {
      return readingGoal - lastReading;
    }

    // Caso direita
    return readingGoal + lastReading;
  }

  setGain(result / numOfTrueSensors);
  //Serial.println(result /numOfTrueSensors);
  outOfLine = false;
  //Retorna a média dos sensores ativos
  lastReading = result / numberOfSensors;
  return result / numOfTrueSensors;
}

void LineFollowerAlgorithm::setGain(float _result)
{
  // Almost on line
  if(almostEqual(_result, readingGoal, maxCenterOffset))
  {
    onLineTime++;
    if(onLineTime > cyclesOnLine)
      setPidHighValue();
  }
  else
  {
    onLineTime = 0;
    setPidLowValue();
  }
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

void LineFollowerAlgorithm::saveCalibration()
{
  for(int i = 0; i < global::numberOfSensor; i++)
  {
    EEPROM.write(i, sensorArr[i].midPoint / 1000);
    EEPROM.commit();

  }
}

void LineFollowerAlgorithm::loadCalibration()
{
  EEPROM.begin(global::numberOfSensor);

  for(int i = 0; i < global::numberOfSensor; i++)
  {
    sensorArr[i].midPoint = EEPROM.read(i) * 1000;
    Serial.println (sensorArr[i].midPoint);
  }

}