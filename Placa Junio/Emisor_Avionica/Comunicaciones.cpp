#include "Comunicaciones.h"
#include <string.h>
#include <stdio.h>
#include "Config.h"
#include "Globals.h"

#include <math.h>
#include <SPI.h>
#include <SdFat.h>
#include "LoRa_E220.h"
#include <TinyGPSPlus.h>
#include <rtos.h>

using namespace rtos;

// Protocolo v1: magic[4], version[1], longitud[1], secuencia LE[2],
// Telemetria[54], CRC16-CCITT-FALSE LE[2]. Total: 64 bytes.
static_assert(sizeof(float) == 4 && sizeof(Telemetria) == 54,
              "Formato de Telemetria incompatible");
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

static void construirTrama(const Telemetria &t, uint16_t secuencia, uint8_t *salida) {
    memcpy(salida, CABECERA, 4);
    salida[4] = 1;
    salida[5] = sizeof(Telemetria);
    salida[6] = (uint8_t)secuencia;
    salida[7] = (uint8_t)(secuencia >> 8);
    memcpy(salida + 8, &t, sizeof(t));
    uint16_t crc = crcTrama(salida, TAM_TRAMA - 2);
    salida[62] = (uint8_t)crc;
    salida[63] = (uint8_t)(crc >> 8);
}

static bool radioLista = false;
static uint16_t secuenciaTX = 0;

// Bus SPI dedicado para SdFat
static arduino::MbedSPI SD_SPI(D9, D11, D10); 
static SdFat SD; 
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &SD_SPI)
static File logFile; // Archivo mantenido abierto para máxima velocidad

// Instancia UART y LoRa
static UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);
static LoRa_E220 e220ttl(&loraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);

static ResponseStatus enviarUARTByteAByte(const uint8_t *datos, size_t longitud) {
    ResponseStatus resultado;
    resultado.code = E220_SUCCESS;

    // Timeout reducido a 150ms para no bloquear la tarea de telemetría si la radio falla
    unsigned long inicio = millis();
    while (digitalRead(LORA_AUX_PIN) == LOW) {
        if (millis() - inicio >= 150) {
            resultado.code = ERR_E220_TIMEOUT;
            return resultado;
        }
        ThisThread::sleep_for(std::chrono::milliseconds(1));
    }

    for (size_t i = 0; i < longitud; ++i) {
        if (loraSerial.write((uint8_t)datos[i]) != 1) {
            resultado.code = ERR_E220_DATA_SIZE_NOT_MATCH;
            return resultado;
        }
    }

    loraSerial.flush();
    ThisThread::sleep_for(std::chrono::milliseconds(2));

    inicio = millis();
    while (digitalRead(LORA_AUX_PIN) == LOW) {
        if (millis() - inicio >= 150) {
            resultado.code = ERR_E220_TIMEOUT;
            return resultado;
        }
        ThisThread::sleep_for(std::chrono::milliseconds(1));
    }

    return resultado;
}

static TinyGPSPlus gps;

static float aleatorioFloat(float minimo, float maximo, int decimales = 3) {
    float escala = pow(10, decimales);
    long min_int = (long)(minimo * escala);
    long max_int = (long)(maximo * escala);
    return random(min_int, max_int + 1) / escala;
}

void inicializarComunicaciones() {
    D_BEGIN(BAUDRATE_DEBUG); 
    Serial1.begin(BAUDRATE_GPS);      

    if (!lora_habilitado) {
        D_PRINTLN("[INFO] LoRa DESACTIVADO. Omitiendo inicio del hardware.");
        return;
    }

    loraSerial.begin(BAUDRATE_LORA_UART);
    D_PRINTLN("Iniciando modulo LoRa E220...");

    if (!e220ttl.begin()) {
        D_PRINTLN("[LoRa] ERROR en begin.");
        return;
    }
    ResponseStructContainer c = e220ttl.getConfiguration();
    if (c.status.code != 1 || c.data == nullptr) {
        D_PRINTLN("[LoRa] ERROR al leer configuracion.");
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
        e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    }

    if (e220ttl.setMode(MODE_0_NORMAL) != 1) {
        D_PRINTLN("[LoRa] ERROR al entrar en modo normal.");
        return;
    }

    radioLista = true;
    D_PRINTLN("[LoRa] Inicializado OK.");
}

void inicializarSD() {
    D_PRINTLN("Inicializando tarjeta SD...");
    if (SD.begin(SD_CONFIG)) {
        // Abrir el archivo una sola vez al inicio
        logFile = SD.open("vuelo.json", FILE_WRITE);
        if (logFile) {
            D_PRINTLN("SD y vuelo.json listos para escritura.");
            sd_disponible = true;
        } else {
            D_PRINTLN("ERROR: No se pudo abrir vuelo.json.");
            sd_disponible = false;
        }
    } else {
        D_PRINTLN("ERROR: Fallo al inicializar SD.");
        sd_disponible = false;
    }
}

void taskGPS() {
    while (true) {
#ifndef SIMULAR_GPS_NO_MONTADO
        while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }
        ThisThread::sleep_for(std::chrono::milliseconds(2)); 
#else
        ThisThread::sleep_for(std::chrono::milliseconds(200));
#endif
    }
}

void taskTelemetria() {
    static uint8_t sync_count = 0;
    char jsonBuffer[256];

    while (true) {
#ifdef SIMULAR_GPS_NO_MONTADO
        latitud  = aleatorioFloat(41.640000, 41.660000, 6);
        longitud = aleatorioFloat(-0.900000, -0.870000, 6);
        vel_horizontal = aleatorioFloat(0.0, 300.0, 2); 
#else
        if (gps.location.isValid()) {
            latitud = gps.location.lat();
            longitud = gps.location.lng();
        } else {
            latitud = 0.0;
            longitud = 0.0;
        }

        if (gps.speed.isValid()) {
            vel_horizontal = gps.speed.mps(); 
        } else {
            vel_horizontal = 0.0;
        }
#endif

        if (vel_horizontal > vel_max) {
            vel_max = vel_horizontal;
        }

        // 1. Estructura de telemetría binaria LoRa
        Telemetria t;
        t.acc_x = acc_x;
        t.acc_y = acc_y;
        t.acc_z = acc_z;
        t.pitch = pitch;
        t.roll = roll;
        t.yaw = yaw;
        t.vel = vel_horizontal;
        t.vel_max = vel_max;
        t.latitud = latitud;
        t.longitud = longitud;
        t.altura = (int16_t)altura_filtrada;
        t.altura_max = (int16_t)altura_max;
        t.vel_vert = vel_vertical;
        t.temperatura = (int16_t)temperatura;
        t.estado = (int16_t)estado_vuelo;
        t.flag = (int16_t)flag_continuidad;

        // 2. Transmisión LoRa
        if (lora_habilitado && radioLista) {
            uint8_t trama[TAM_TRAMA];
            construirTrama(t, secuenciaTX++, trama);
            enviarUARTByteAByte(trama, sizeof(trama));
        }

        // 3. Registro local en MicroSD (Usa buffer estático, sin String ni malloc)
        if (sd_disponible && logFile) {
            snprintf(jsonBuffer, sizeof(jsonBuffer),
                "{\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,"
                "\"vel\":%.2f,\"vel_max\":%.2f,\"latitud\":%.6f,\"longitud\":%.6f,\"altura\":%.2f,"
                "\"altura_max\":%.2f,\"vel_vert\":%.2f,\"temperatura\":%.2f,\"estado\":%d,\"flag\":%d}",
                acc_x, acc_y, acc_z, pitch, roll, yaw,
                vel_horizontal, vel_max, latitud, longitud,
                altura_filtrada, altura_max, vel_vertical, temperatura,
                estado_vuelo, flag_continuidad
            );

            D_PRINTLN(jsonBuffer);
            logFile.println(jsonBuffer);

            // Sincronizar físicamente a la SD cada 1 segundo (cada 10 muestras de 100ms)
            if (++sync_count >= 10) {
                logFile.flush();
                sync_count = 0;
            }
        }

        ThisThread::sleep_for(std::chrono::milliseconds(INTERVALO_ENVIO_MS));
    }
}