#ifndef REMOTESETUP_H  
#define REMOTESETUP_H

#include <Arduino.h>
#include <BluetoothSerial.h>

#include "LineFollowerAlgorithm.h"

class Remote
{
public:
    Remote(LineFollowerAlgorithm& _lineFollower){lineFollower = &_lineFollower;}
    
    void beginBluetooth();

    void sendAnalogRead();
    void sendMidPoint();
    void sendPID();
    void changePID(float variation, char c);
    void process();


    BluetoothSerial bluetoothModule;
    LineFollowerAlgorithm* lineFollower;
private:
    bool setupMode = false;
    char mode = 'p';
    bool lowGainMode = true;


};

// "!" (33) - Transmitir leitura
// "#" (35) - Iniciar
// "%" (37) - Parar
// "'" (39) - Calibrar
// ")" (41) - Encerrar calibração
// "+" (43) - Atualizar média
// "-" (45) - Modo setup
// "." (46) - Modo Baixo Ganho
// "/" (47) - Modo Alto Ganho

// "1" (49) - Nivel 1
// "2" (50) - Nivel 2
// "3" (51) - Nivel 3



// " " " (34) - Modo P
// "$" (36) - Modo I
// "&" (38) - Modo D
// "(" (40) - Modo M
// "*" (42) - Atualizar PID
// "," (44) - Sair do setup
// "a" (97) - -0.001
// "b" (98) - -0.01
// "c" (99) - -0.1
// "d" (100) - +0.1
// "e" (101) - +0.01
// "f" (102) - +0.001



#endif