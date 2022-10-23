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
  digitalWrite(ledPin, HIGH);
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
  pinMode(rightSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  for(int i = 0; i < numberOfSensors; i++)
    sensorArr[i].lineIsBlack = lineIsBlack;
    //loadCalibration();
}

void LineFollowerAlgorithm::run()
{
  //checkLineColor();
  float sensVal = calculateSensValue();

  float leftMotor = gain;
  float rightMotor = gain;
  leftMotor += pidLeft.calculate(readingGoal - sensVal);
  rightMotor += pidRight.calculate(sensVal - readingGoal);


  if(leftMotor > motorLimiter)
    leftMotor = motorLimiter;
  if(leftMotor < -motorLimiter)
    leftMotor = -motorLimiter;
  if(rightMotor > motorLimiter)
    rightMotor = motorLimiter;
  if(rightMotor < -motorLimiter)
    rightMotor = -motorLimiter;
  
  
  //Serial.print(sensVal);
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
      errorMultiplier = float(i);
      

      switch (i)
      {
      case 0:
        errorMultiplier = 0;
        break;
      case 1:
        errorMultiplier = 1;
        break;
      case 2:
        errorMultiplier = 2;
        break;
      case 3:
        errorMultiplier = 3;
        break;
      case 4:
        errorMultiplier = 4;
        break;
      case 5:
        errorMultiplier = 5;
        break;
      case 6:
        errorMultiplier = 6;
        break;    
      case 7:
        errorMultiplier = 7;
        break;   
      case 8:
        errorMultiplier = 8;
        break;   
      case 9:
        errorMultiplier = 9;
        break;  
      case 10:
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
  if(numOfTrueSensors >= 5)
    lastCrossing = millis();

  //Serial.println(lastReading);

  // Fazendo a volta se perder os sensores
  if(numOfTrueSensors == 0 || numOfTrueSensors == numberOfSensors)
  {
    outOfTrack = true;
    // Caso a ultima leitura tenha sido mais ou menos no centro
    if(almostEqual(lastReading, readingGoal, maxCenterOffset))
    {
      return readingGoal;
    }
    // Caso a ultima leitura tenha sido para a esquerda
    if(lastReading < readingGoal)
    {
      outOfLine = true;
      return 0;
    }
    if(readingGoal > lastReading)
    {
      // Caso direita
      outOfLine = true;
      return (float)numberOfSensors;
    }



  }
  if( numOfTrueSensors)
    setGain(result / numOfTrueSensors);
  outOfTrack = false;
  outOfLine = false;

  //Retorna a média dos sensores ativos
  if(!numberOfSensors)
  {
    
  lastReading = 0;
  return 0;

  }
  lastReading = ((result / numOfTrueSensors) + lastReading)/2;
  return result / numOfTrueSensors;
}

void LineFollowerAlgorithm::setGain(float _result)
{
  if(outOfLine && outOfTrack)
  {
    setPidOffLineValue();
    onLineTime = 0;
    return;
  }
  // Almost on line
  if(almostEqual(_result, readingGoal, maxCenterOffset))
  {
    onLineTime++;
    if(onLineTime > cyclesOnLine)
      setPidHighValue();
    return;
  }
  if(!outOfTrack)
  {
    onLineTime = 0;
    setPidLowValue();
    return;
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
  if(!isCallibrating)
  digitalWrite(ledPin, LOW);
  if(isCallibrating)
    calibrateSensors();

  if(digitalRead(rightSensorPin))
  {
    if(millis() - 2000 > lastCrossing)
      isRunning = false;
  }

  if(isRunning)
    run();
  
  if(!isRunning)
    motorController -> driveMotor(0, 0);

}

void LineFollowerAlgorithm::saveCalibration()
{
  return;
  for(int i = 0; i < global::numberOfSensor; i++)
  {
    EEPROM.write(i, sensorArr[i].midPoint / 1000);
    EEPROM.commit();

  }
}

void LineFollowerAlgorithm::loadCalibration()
{
  return;
  EEPROM.begin(global::numberOfSensor);

  for(int i = 0; i < global::numberOfSensor; i++)
  {
    sensorArr[i].midPoint = EEPROM.read(i) * 1000;
    Serial.println (sensorArr[i].midPoint);
  }

}