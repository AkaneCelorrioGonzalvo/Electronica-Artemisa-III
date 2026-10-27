#include "Globals.h"

// ============================================================
// ACTIVACIÓN DE HARDWARE
// ============================================================
// Cambia a 'true' en cuanto tengáis y conectéis los módulos LoRa E220
bool lora_habilitado = false; 
bool sd_disponible   = true;

FiltroKalman1D filtroAltura;

// Inicialización de variables
float acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
float pressure_kPa = 101.325;
float pressure_inicial = 101.325;
float temperatura = 25.0;

float pitch = 0.0;
float roll  = 0.0;
float yaw   = 0.0;

float latitud = 0.0;
float longitud = 0.0;
float vel_horizontal = 0.0;
float vel_max = 0.0;

float altura_filtrada = 0.0;
float altura_max = -9999.0;
float vel_vertical = 0.0;

int estado_vuelo = 0;
int flag_continuidad = 0;
int numeroTransmision = 0;