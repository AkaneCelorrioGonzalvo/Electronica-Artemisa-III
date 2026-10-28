#pragma once
#include <Arduino.h>

// ============================================================
// CONFIGURACIÓN DE PRUEBAS / SIMULACIÓN / DEPURACIÓN
// ============================================================
//#define SIMULAR_GPS_NO_MONTADO

// Comenta la siguiente línea antes del lanzamiento para desactivar USB
#define DEBUG_USB

#ifdef DEBUG_USB
  #define D_BEGIN(baud)   Serial.begin(baud)
  #define D_PRINT(...)    Serial.print(__VA_ARGS__)
  #define D_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define D_BEGIN(baud)
  #define D_PRINT(...)
  #define D_PRINTLN(...)
#endif

// ============================================================
// CONFIGURACIÓN DE PINES (PCB PERSONALIZADA)
// ============================================================
constexpr int PIN_PARACAIDAS       = D7;
constexpr int PIN_SENSE_PARACAIDAS = A0;
constexpr int PIN_BUZZER           = D13;
constexpr int PIN_SD_CS            = D12;

// Pines LoRa E220
#define LORA_AUX_PIN A3
#define LORA_RX_PIN  A4
#define LORA_TX_PIN  A5
#define LORA_M1_PIN  A6
#define LORA_M0_PIN  A7

// ============================================================
// VELOCIDADES Y TIEMPOS
// ============================================================
constexpr uint32_t BAUDRATE_DEBUG     = 115200;
constexpr uint32_t BAUDRATE_GPS       = 9600;
constexpr uint32_t INTERVALO_ENVIO_MS = 100;