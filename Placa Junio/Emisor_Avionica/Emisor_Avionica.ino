#include "Config.h"
#include "Globals.h"
#include "Sensores.h"
#include "ControlVuelo.h"
#include "Comunicaciones.h"

#include <rtos.h>

using namespace rtos;

// ============================================================
// HILOS (THREADS) DE RTOS
// ============================================================
Thread hilosSensores(osPriorityAboveNormal);    
Thread hiloGPS(osPriorityNormal);              
Thread hiloControlVuelo(osPriorityHigh);       
Thread hiloTelemetria(osPriorityBelowNormal);   

void setup() {
    delay(2000);
    
    D_PRINTLN("Inicializando Comunicaciones...");
    inicializarComunicaciones();

    inicializarSD();
    inicializarSensores();
    inicializarControlVuelo();

    randomSeed(analogRead(A1)); 

    // Arranque de los hilos de ejecución
    hilosSensores.start(taskSensores);
    hiloGPS.start(taskGPS);
    hiloControlVuelo.start(taskControlVuelo);
    hiloTelemetria.start(taskTelemetria);
}

void loop() {
    // Dormir el hilo principal para liberar la CPU
    ThisThread::sleep_for(std::chrono::milliseconds(1000)); 
}