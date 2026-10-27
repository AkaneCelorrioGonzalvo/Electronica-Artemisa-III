#include "ControlVuelo.h"
#include "Config.h"
#include "Globals.h"

#include <math.h>
#include <rtos.h>

using namespace rtos;

static uint32_t tiempo_anterior_calc_vel = 0;
static uint32_t tiempo_disparo_paracaidas = 0;
static float altura_anterior = 0.0;
static float vel_vertical_filtrada = 0.0;

void inicializarControlVuelo() {
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

            vel_vertical_filtrada = (0.75 * vel_vertical_filtrada) + (0.25 * vel_vertical_inst);
            vel_vertical = vel_vertical_filtrada;
        }

        altura_anterior = altura_filtrada;
        tiempo_anterior_calc_vel = tiempo_actual;

        int lectura_sense = analogRead(PIN_SENSE_PARACAIDAS);
        flag_continuidad = (lectura_sense > 500) ? 1 : 0;

        switch (estado_vuelo) {
            case 0: 
                digitalWrite(PIN_PARACAIDAS, LOW);
                if (altura_filtrada > 10.0) {
                    estado_vuelo = 1;
                }
                break;

            case 1: 
                digitalWrite(PIN_PARACAIDAS, LOW);
                if (altura_filtrada > 15.0 && vel_vertical < -1.0) {
                    digitalWrite(PIN_PARACAIDAS, HIGH); 
                    tiempo_disparo_paracaidas = millis();
                    estado_vuelo = 2; 
                }
                break;

            case 2: 
                if (millis() - tiempo_disparo_paracaidas >= 2000) {
                    digitalWrite(PIN_PARACAIDAS, LOW); 
                }
                if (altura_filtrada < 3.0 && abs(vel_vertical) < 0.2) {
                    estado_vuelo = 3;
                }
                break;

            case 3: 
                digitalWrite(PIN_PARACAIDAS, LOW);
                break;
        }

        if (estado_vuelo == 3) {
            if ((millis() / 500) % 2 == 0) {
                digitalWrite(PIN_BUZZER, HIGH);
            } else {
                digitalWrite(PIN_BUZZER, LOW);
            }
        } else {
            digitalWrite(PIN_BUZZER, LOW); 
        }

        ThisThread::sleep_for(std::chrono::milliseconds(20));
    }
}