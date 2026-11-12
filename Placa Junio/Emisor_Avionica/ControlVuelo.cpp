#include "ControlVuelo.h"
#include "Config.h"
#include "Globals.h"

#include <math.h>
#include <rtos.h>

using namespace rtos;

static uint32_t tiempo_anterior_calc_vel = 0;
static uint32_t tiempo_disparo_paracaidas = 0;
static uint32_t tiempo_despegue = 0;
static float altura_anterior = 0.0;
static float vel_vertical_filtrada = 0.0;
static int contador_suelo = 0;

void inicializarControlVuelo() {
    pinMode(PIN_PARACAIDAS_1, OUTPUT);
    digitalWrite(PIN_PARACAIDAS_1, LOW); 

    pinMode(PIN_PARACAIDAS_2, OUTPUT);
    digitalWrite(PIN_PARACAIDAS_2, LOW); 
    
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); 
    
    pinMode(PIN_SENSE_PARACAIDAS_1, INPUT);

    pinMode(PIN_DEBUG_1, OUTPUT);
    pinMode(PIN_DEBUG_2, OUTPUT);
    pinMode(PIN_DEBUG_3, OUTPUT);
    pinMode(PIN_DEBUG_4, OUTPUT);
    pinMode(PIN_DEBUG_5, OUTPUT);
    digitalWrite(PIN_DEBUG_1, LOW);
    digitalWrite(PIN_DEBUG_2, LOW);
    digitalWrite(PIN_DEBUG_3, LOW);
    digitalWrite(PIN_DEBUG_4, LOW);
    digitalWrite(PIN_DEBUG_5, LOW);

    tiempo_anterior_calc_vel = millis();
}

void taskControlVuelo() {
    tiempo_anterior_calc_vel = millis();

    while (true) {
        float altura_relativa_raw = 44330.0 * (1.0 - pow(pressure_kPa / pressure_inicial, 1.0 / 5.255));
        
        altura_filtrada = filtroAltura.actualizar(altura_relativa_raw);

        if (altura_max == -9999.0) {
            altura_max = altura_filtrada;
        } else if (altura_filtrada > altura_max) {
            altura_max = altura_filtrada;
        }

        uint32_t tiempo_actual = millis();
        float dt = (tiempo_actual - tiempo_anterior_calc_vel) / 1000.0; 
        
        if (dt > 0.0) {
            float d_altura = altura_filtrada - altura_anterior; 
            float vel_vertical_inst = d_altura / dt; 

            vel_vertical_filtrada = (0.75f * vel_vertical_filtrada) + (0.25f * vel_vertical_inst);
            vel_vertical = vel_vertical_filtrada;
        }

        altura_anterior = altura_filtrada;
        tiempo_anterior_calc_vel = tiempo_actual;

        // Comprobación de continuidad de la carga en A0
        int lectura_sense = analogRead(PIN_SENSE_PARACAIDAS_1);
        flag_continuidad = (lectura_sense > 500) ? 1 : 0;

        switch (estado_vuelo) {
            case 0: // En rampa / espera
                digitalWrite(PIN_PARACAIDAS_1, LOW);
                digitalWrite(PIN_PARACAIDAS_2, LOW);
                
                // Despegue: altura > 4m O aceleración axial fuerte de empuje (<= -2.0G)
                if (altura_filtrada > 4.0 || acc_x < -2.0f) {
                    tiempo_despegue = millis();
                    estado_vuelo = 1; // Despegue detectado
                }
                break;

            case 1: // Ascenso
                digitalWrite(PIN_PARACAIDAS_1, LOW);
                
                // Seguridad: no comprobar apogeo en los primeros 2.0s de quema del motor
                if ((millis() - tiempo_despegue > 2000) && altura_filtrada > 15.0 && vel_vertical < -1.0) {
                    #if HABILITAR_DISPARO_PIROTECNIA
                    digitalWrite(PIN_PARACAIDAS_1, HIGH); 
                    #endif
                    tiempo_disparo_paracaidas = millis();
                    estado_vuelo = 2; // Apogeo detectado
                }
                break;

            case 2: // Descenso
                if (millis() - tiempo_disparo_paracaidas >= 250) {
                    digitalWrite(PIN_PARACAIDAS_1, LOW); 
                }
                
                // Aterrizaje robusto contra bias: umbral en 5.5m y velocidad vertical baja estable
                if (altura_filtrada < 5.5 && abs(vel_vertical) < 0.8) {
                    contador_suelo++;
                    // 50 ciclos x 20ms = 1 segundo continuo quieto en tierra
                    if (contador_suelo >= 50) {
                        estado_vuelo = 3; // Aterrizaje confirmado
                    }
                } else {
                    contador_suelo = 0;
                }
                break;

            case 3: // En tierra
                digitalWrite(PIN_PARACAIDAS_1, LOW);
                digitalWrite(PIN_PARACAIDAS_2, LOW);
                break;
        }

        // Buzzer intermitente post-aterrizaje para rescate
        /*
        if (estado_vuelo == 3) {
            
            if ((millis() / 500) % 2 == 0) {
                digitalWrite(PIN_BUZZER, HIGH);
            } else {
                digitalWrite(PIN_BUZZER, LOW);
            }
        } else {
            digitalWrite(PIN_BUZZER, LOW); 
        }*/

        ThisThread::sleep_for(std::chrono::milliseconds(20));
    }
}