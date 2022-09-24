#include "RemoteSetup.h"

void Remote::beginBluetooth()
{
    bluetoothModule.begin("SEGUIDOR_ESP32");
    Serial.println("Bluetooth Device is Ready to Pair");

}

void Remote::sendAnalogRead()
{
    if(lineFollower->isRunning)
        return;
    bluetoothModule.print(lineFollower->sensorArr[0].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[1].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[2].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[3].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[4].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[5].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[6].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[7].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[8].readValueAnalog());
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[9].readValueAnalog());
    bluetoothModule.println();
}

void Remote::sendMidPoint()
{
    if(lineFollower->isRunning)
        return;
    bluetoothModule.print(lineFollower->sensorArr[0].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[1].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[2].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[3].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[4].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[5].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[6].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[7].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[8].midPoint);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->sensorArr[9].midPoint);
    bluetoothModule.println();
}

void Remote::sendPID()
{
    if(Remote::lowGainMode)
    {
        bluetoothModule.print(lineFollower->pidLow.p, 3);
        bluetoothModule.print(",");
        bluetoothModule.print(lineFollower->pidLow.i * 10000, 3);
        bluetoothModule.print(",");
        bluetoothModule.print(lineFollower->pidLow.d, 3);
        bluetoothModule.print(",");
        bluetoothModule.print(lineFollower->lowGain, 3);
        bluetoothModule.println();

        Serial.print("LOW PID: ");
        Serial.print(lineFollower->pidLow.p,3);
        Serial.print(",");
        Serial.print(lineFollower->pidLow.i,7);
        Serial.print(",");
        Serial.print(lineFollower->pidLow.d,3);
        Serial.print(",");
        Serial.print(lineFollower->lowGain,3);
        Serial.println();
        return;
    }
    bluetoothModule.print(lineFollower->pidHigh.p, 3);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->pidHigh.i * 10000, 3);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->pidHigh.d, 3);
    bluetoothModule.print(",");
    bluetoothModule.print(lineFollower->highGain, 3);
    bluetoothModule.println();

    Serial.print("HIGH PID: ");
    Serial.print(lineFollower->pidHigh.p,3);
    Serial.print(",");
    Serial.print(lineFollower->pidHigh.i,7);
    Serial.print(",");
    Serial.print(lineFollower->pidHigh.d,3);
    Serial.print(",");
    Serial.print(lineFollower->highGain,3);
    Serial.println();


}

void Remote::changePID(float variation, char c)
{
    switch (c)
    {
    case 'p':
        if(Remote::lowGainMode)
            lineFollower->pidLow.p += variation;
        else
            lineFollower->pidHigh.p += variation;
        break;
    case 'i':
        if(Remote::lowGainMode)
            lineFollower->pidLow.i += variation/1000;
        else
            lineFollower->pidHigh.i += variation/1000;
        break;
    case 'd':
        if(Remote::lowGainMode)
            lineFollower->pidLow.d += variation;
        else
            lineFollower->pidHigh.d += variation;
        break;
    case 'm':
        if(Remote::lowGainMode)
            lineFollower->lowGain += variation;
        else
            lineFollower->highGain += variation;
        break;
    
    default:
        break;
    }
}

void Remote::process()
{
    if (bluetoothModule.available()) //Check if we receive anything from Bluetooth
    {
        int incoming = bluetoothModule.read();
        Serial.print("Received:"); 
        Serial.println(incoming);

        if(setupMode)
        {
        switch (0) //incoming
        {
        case 34: // " " " (34) - Modo P
            mode = 'p';
            break;
        case 36: // "$" (36) - Modo I
            mode = 'i';
            break;
        case 38: // "&" (38) - Modo D
            mode = 'd';
            break;
        case 40: // "(" (40) - Modo M
            mode = 'm';
            break;
        case 42: // "*" (42) - Atualizar PID
            sendPID();
            break;
        case 44: // "," (44) - Sair do setup
            setupMode = false;
            break;
        case 46: // "." (46) - Modo Baixo Ganho
            lowGainMode = true;
            //lineFollower->setPidLowValue();
            break;
        case 47: // "/" (47) - Modo Alto Ganho
            lowGainMode = false;
            //lineFollower->setPidHighValue();
            break;
        case 97: // "a" (97) - -0.001
            changePID(-0.001, mode);
            sendPID();
            break;
        case 98: // "b" (98) - -0.01
            changePID(-0.01, mode);
            sendPID();
            break;
        case 99: // "c" (99) - -0.1
            changePID(-0.1, mode);
            sendPID();
            break;
        case 100: // "d" (100) - 0.1
            changePID(0.1, mode);
            sendPID();
            break;
        case 101: // "e" (101) - 0.01
            changePID(0.01, mode);
            sendPID();
            break;
        case 102: // "f" (102) - 0.001
            changePID(0.001, mode);
            sendPID();
            break;

        }
        }
        else if(canChange)
        {
        switch (incoming)
        {
        case 33: // "!" (33) - Transmitir leitura
            if(lineFollower->isRunning)
                break;
            sendAnalogRead();
            lineFollower->printAllSensorsAnalog();

            break;

        case 35: // "#" (35) - Iniciar
            Serial.println("Iniciar"); 
            lineFollower->isRunning = true;
            break;

        case 37: // "%" (37) - Parar
            Serial.println("Parar"); 
            canChange=false;
            //lineFollower->isRunning = false;
            break;

        case 39: // "'" (39) - Calibrar
            Serial.println("Calibrar"); 
            lineFollower->isCallibrating = true;
            Serial.println(",");

            break;

        case 41: // ")" (41) - Encerrar calibração
            Serial.println("Encerrar calibracao"); 
            lineFollower->isCallibrating = false;
            break;

        case 43: // "+" (43) - Atualizar média
            Serial.println("Atualizar");
            sendMidPoint();
            break;      
        
        case 45: // "-" (45) - Modo setup
            Serial.println("Setup");
            setupMode = true;
            break;

        case 49: // "1" (49) - Nivel 1
            Serial.println("Nivel 1");
            break;

        case 50: // "2" (50) - Nivel 2
            Serial.println("Nivel 2");

            break;

        case 51: // "3" (51) - Nivel 3
            Serial.println("Nivel 3");

            break;

        case 114: // "r" (114) - restartMotor
            Serial.println("Restart");
            lineFollower->motorController->restartMotor();

            break;
        }
        }
    

    }
}