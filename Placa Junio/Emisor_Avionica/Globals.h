#pragma once
#include <Arduino.h>
#include <stdint.h>

// ============================================================
// ESTRUCTURAS DE DATOS
// ============================================================
struct __attribute__((packed)) Telemetria {
    float acc_x;
    float acc_y;
    float acc_z;
    float pitch;
    float roll;
    float yaw;
    float vel;
    float vel_max;
    float latitud;
    float longitud;
    int16_t altura;
    int16_t altura_max;
    float vel_vert;
    int16_t temperatura;
    int16_t estado;
    int16_t flag;
};

struct FiltroKalman1D {
    float q; 
    float r; 
    float x; 
    float p; 
    float k; 

    void inicializar(float x_inicial, float ruido_proceso, float ruido_medida) {
        x = x_inicial;
        q = ruido_proceso;
        r = ruido_medida;
        p = 1.0;
    }

    float actualizar(float medida) {
        p = p + q;
        k = p / (p + r);
        x = x + k * (medida - x);
        p = (1.0 - k) * p;
        return x;
    }
};

// ============================================================
// CONTROL DE HARDWARE Y DISPONIBILIDAD
// ============================================================
extern bool lora_habilitado;
extern bool sd_disponible;

// ============================================================
// DECLARACIÓN 'EXTERN' DE VARIABLES COMPARTIDAS
// ============================================================
extern FiltroKalman1D filtroAltura;

// Sensores internos
extern float acc_x, acc_y, acc_z;
extern float gyro_x, gyro_y, gyro_z;
extern float mag_x, mag_y, mag_z;
extern float pressure_kPa;
extern float pressure_inicial;
extern float temperatura;

// Orientación
extern float pitch;
extern float roll;
extern float yaw;

// GPS y Cinemática
extern float latitud;
extern float longitud;
extern float vel_horizontal;
extern float vel_max;

// Altura y vertical
extern float altura_filtrada;
extern float altura_max;
extern float vel_vertical;

// Control de vuelo
extern int estado_vuelo;
extern int flag_continuidad;
extern int numeroTransmision;