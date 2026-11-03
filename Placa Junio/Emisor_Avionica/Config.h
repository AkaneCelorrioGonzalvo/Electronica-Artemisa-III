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
// SEGURIDAD DE PIROTECNIA
// ============================================================
// En este cohete no se usa expulsión pirotécnica activa.
// Mantener en 'false' garantiza que D8 y D7 permanezcan en LOW estricto.
#define HABILITAR_DISPARO_PIROTECNIA false

// ============================================================
// CONFIGURACIÓN DE PINES (PCB PERSONALIZADA)
// ============================================================
// Paracaídas y detección
constexpr int PIN_SENSE_PARACAIDAS_1 = A0; // No usado en este lanzamiento
constexpr int PIN_SENSE_PARACAIDAS_2 = A1; // No usado en este lanzamiento
constexpr int PIN_PARACAIDAS_1       = D8; // Actuador 1
constexpr int PIN_PARACAIDAS_2       = D7; // Actuador 2
constexpr int PIN_SENSE_ARMADO       = D13; // Reservado

// Avisador acústico
constexpr int PIN_BUZZER             = A2;

// Bus MicroSD (MbedSPI dedicado: D9=MISO, D11=MOSI, D10=CLK)
constexpr int PIN_SD_CS              = D12;

// Radio LoRa E220
#define LORA_AUX_PIN A3
#define LORA_RX_PIN  A4 // RX del Arduino (conectado al TX del LoRa)
#define LORA_TX_PIN  A5 // TX del Arduino (conectado al RX del LoRa)
#define LORA_M1_PIN  A6
#define LORA_M0_PIN  A7

// Pines libres de DEBUG (D2 - D6)
constexpr int PIN_DEBUG_1            = D2; // Sugerencia: Profiling Sensores o Heartbeat
constexpr int PIN_DEBUG_2            = D3; // Sugerencia: Profiling Control Vuelo o LED LoRa
constexpr int PIN_DEBUG_3            = D4; // Sugerencia: Profiling Telemetria o LED GPS Fix
constexpr int PIN_DEBUG_4            = D5; // Sugerencia: Profiling GPS o LED SD OK
constexpr int PIN_DEBUG_5            = D6; // Sugerencia: Trigger Cámara o Continuidad

// ============================================================
// CONFIGURACIÓN LORA Y COMUNICACIONES
// ============================================================
constexpr uint8_t  LORA_CANAL         = 1; // Cambiar al canal regulado en campo (ej. 19)
constexpr uint32_t BAUDRATE_DEBUG     = 115200;
constexpr uint32_t BAUDRATE_GPS       = 9600;
constexpr uint32_t BAUDRATE_LORA_UART = 115200; 
constexpr uint32_t INTERVALO_ENVIO_MS = 100;