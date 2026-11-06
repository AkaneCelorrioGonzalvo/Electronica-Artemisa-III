#include <Arduino.h>
#include "LoRa_E220.h"
#include <string.h>
#include <math.h>

// ============================================================
// CONFIGURACIÓN DE PINES (ESP32-S3-Zero)
// ============================================================
#define RX_PIN  9   // Pin RX del ESP32-S3 (al TX del LoRa)
#define TX_PIN  10  // Pin TX del ESP32-S3 (al RX del LoRa)
#define AUX_PIN 6
#define M1_PIN  7
#define M0_PIN  8

// Canal de radio oficial (debe coincidir con el emisor: canal 19)
constexpr uint8_t LORA_CANAL = 19;

// Si necesitas ver mensajes de depuración en el Monitor Serie de Arduino sin Python, descomenta la siguiente línea:
//#define DEBUG_SERIAL

// Inicialización del LoRa en Serial2 del ESP32-S3
LoRa_E220 e220ttl(&Serial2, AUX_PIN, M0_PIN, M1_PIN);

// ============================================================
// ESTRUCTURA DE TELEMETRÍA (Exactamente 54 Bytes)
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

// Protocolo v1: Cabecera[4] + Version[1] + Longitud[1] + Secuencia[2] + Telemetria[54] + CRC[2] = 64 Bytes
static_assert(sizeof(float) == 4 && sizeof(Telemetria) == 54, "Formato de Telemetria incompatible");
static const size_t TAM_TRAMA = 64;
static const uint8_t CABECERA[4] = {0xA5, 0x5A, 0xC3, 0x3C};

static uint16_t crcTrama(const uint8_t *datos, size_t longitud) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < longitud; ++i) {
        crc ^= (uint16_t)datos[i] << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool radioLista = false;

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(2000); // Espera para enumeración del USB nativo del ESP32-S3

    radioLista = false;

    // Comunicación con el LoRa E220 a 115200 baudios
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    if (!e220ttl.begin()) {
        #ifdef DEBUG_SERIAL
        Serial.println("ERROR: no se ha podido iniciar el E220.");
        #endif
        return;
    }

    ResponseStructContainer c = e220ttl.getConfiguration();
    if (c.status.code != 1 || c.data == nullptr) {
        #ifdef DEBUG_SERIAL
        Serial.println("ERROR leyendo configuracion LoRa.");
        #endif
        return;
    }

    Configuration config = *(Configuration *)c.data;
    c.close();

    bool cambiar = (config.CHAN != LORA_CANAL) ||
                   (config.ADDH != 0) ||
                   (config.ADDL != 0) ||
                   (config.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION) ||
                   (config.TRANSMISSION_MODE.enableRSSI != 0);

    config.CHAN = LORA_CANAL;
    config.ADDH = 0;
    config.ADDL = 0;
    config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    config.TRANSMISSION_MODE.enableRSSI = 0;

    if (cambiar) {
        ResponseStatus resultado = e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
        if (resultado.code != 1) {
            #ifdef DEBUG_SERIAL
            Serial.println("ERROR guardando configuracion.");
            #endif
            return;
        }
    }

    if (e220ttl.setMode(MODE_0_NORMAL) != 1) {
        #ifdef DEBUG_SERIAL
        Serial.println("ERROR activando modo normal.");
        #endif
        return;
    }

    radioLista = true;
}

// ============================================================
// VENTANA DESLIZANTE PARA SINCRONIZACIÓN DE TRAMA
// ============================================================
static uint8_t ventana[TAM_TRAMA];
static size_t usados = 0;

static void descartarPrimero() {
    memmove(ventana, ventana + 1, --usados);
}

static bool recibirByte(uint8_t dato, Telemetria &t, uint16_t &secuencia) {
    ventana[usados++] = dato;
    while (usados > 0) {
        size_t comparar = (usados < 4) ? usados : 4;
        if (memcmp(ventana, CABECERA, comparar) != 0) {
            descartarPrimero();
            continue;
        }
        if (usados < 6) return false;
        if (ventana[4] != 1 || ventana[5] != sizeof(Telemetria)) {
            descartarPrimero();
            continue;
        }
        if (usados < TAM_TRAMA) return false;

        uint16_t recibido = (uint16_t)ventana[62] | ((uint16_t)ventana[63] << 8);
        if (recibido != crcTrama(ventana, 62)) {
            descartarPrimero();
            continue;
        }

        memcpy(&t, ventana + 8, sizeof(t));
        secuencia = (uint16_t)ventana[6] | ((uint16_t)ventana[7] << 8);
        usados = 0;
        return true;
    }
    return false;
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================
void loop() {
    if (!radioLista) {
        delay(10);
        return;
    }

    while (Serial2.available() > 0) {
        int dato = Serial2.read();
        if (dato < 0) break;

        Telemetria t;
        uint16_t secuencia;

        // Comprueba cabecera, versión, longitud y CRC
        if (!recibirByte((uint8_t)dato, t, secuencia)) {
            continue;
        }

        // Filtro de valores no finitos o inválidos
        if (!isfinite(t.acc_x) || !isfinite(t.acc_y) || !isfinite(t.acc_z) ||
            !isfinite(t.pitch) || !isfinite(t.roll)  || !isfinite(t.yaw)   ||
            !isfinite(t.vel)   || !isfinite(t.vel_max) ||
            !isfinite(t.latitud) || !isfinite(t.longitud) || !isfinite(t.vel_vert) ||
            t.estado < 0 || t.estado > 3 || (t.flag != 0 && t.flag != 1)) {
            continue;
        }

        // ============================================================
        // SALIDA ÚNICA POR SERIAL (Formato JSON limpio para Python)
        // ============================================================
        String json = "{";
        //json += "\"secuencia\":" + String(secuencia) + ",";
        json += "\"acc_x\":" + String(t.acc_x, 3) + ",";
        json += "\"acc_y\":" + String(t.acc_y, 3) + ",";
        json += "\"acc_z\":" + String(t.acc_z, 3) + ",";
        json += "\"pitch\":" + String(t.pitch, 2) + ",";
        json += "\"roll\":" + String(t.roll, 2) + ",";
        json += "\"yaw\":" + String(t.yaw, 2) + ",";
        json += "\"vel\":" + String(t.vel, 2) + ",";
        json += "\"vel_max\":" + String(t.vel_max, 2) + ",";
        json += "\"latitud\":" + String(t.latitud, 6) + ",";
        json += "\"longitud\":" + String(t.longitud, 6) + ",";
        json += "\"altura\":" + String(t.altura) + ",";
        json += "\"altura_max\":" + String(t.altura_max) + ",";
        json += "\"vel_vert\":" + String(t.vel_vert, 2) + ",";
        json += "\"temperatura\":" + String(t.temperatura) + ",";
        json += "\"estado\":" + String(t.estado) + ",";
        json += "\"flag\":" + String(t.flag);
        json += "}";

        Serial.println(json); // Envía exactamente 1 línea terminada en \n
    }

    delay(1);
}