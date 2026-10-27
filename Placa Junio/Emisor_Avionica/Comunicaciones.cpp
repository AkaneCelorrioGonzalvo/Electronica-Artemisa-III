#include "Comunicaciones.h"
#include "Config.h"
#include "Globals.h"

#include <math.h>
#include <SPI.h>
#include <SdFat.h>
#include "LoRa_E220.h"
#include <TinyGPSPlus.h>
#include <rtos.h>

using namespace rtos;

// SPI personalizado y SD
static arduino::MbedSPI SD_SPI(D9, D11, D10); 
static SdFat SD; 
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &SD_SPI)

// Instancia LoRa
static UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);
static LoRa_E220 e220ttl(&loraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);

// Instancia GPS
static TinyGPSPlus gps;

// Generador de números aleatorios para simulación
static float aleatorioFloat(float minimo, float maximo, int decimales = 3) {
    float escala = pow(10, decimales);
    long min_int = (long)(minimo * escala);
    long max_int = (long)(maximo * escala);
    return random(min_int, max_int + 1) / escala;
}

void inicializarComunicaciones() {
    DEBUG_BEGIN(BAUDRATE_DEBUG); 
    Serial1.begin(BAUDRATE_GPS);      

    if (!lora_habilitado) {
        DEBUG_PRINTLN("[INFO] LoRa DESACTIVADO (lora_habilitado = false). Omitiendo inicio del hardware.");
        return;
    }

    loraSerial.begin(9600); 

    DEBUG_PRINTLN("Iniciando modulo LoRa E220...");
    e220ttl.begin();

    ResponseStructContainer c = e220ttl.getConfiguration();
    if (c.status.code == 1) {
        Configuration config = *(Configuration*) c.data;
        bool cambiar_config = false;

        if (config.CHAN != 18) {
            config.CHAN = 18;
            cambiar_config = true;
        }
        if (config.ADDH != 0x00) {
            config.ADDH = 0x00;
            cambiar_config = true;
        }
        if (config.ADDL != 0x00) {
            config.ADDL = 0x00;
            cambiar_config = true;
        }
        if (config.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION) {
            config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
            cambiar_config = true;
        }

        if (cambiar_config) {
            DEBUG_PRINTLN("Actualizando la configuracion del LoRa (Emisor)...");
            e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
            DEBUG_PRINTLN("¡Configuracion del Emisor guardada con exito!");
        } else {
            DEBUG_PRINTLN("La configuracion del LoRa ya es correcta en el Emisor.");
        }
    } else {
        #ifdef DEBUG_USB
        DEBUG_PRINT("No se pudo leer la configuracion del LoRa: ");
        DEBUG_PRINTLN(c.status.getResponseDescription());
        #endif
    }

    e220ttl.setMode(MODE_0_NORMAL);
}

void inicializarSD() {
    DEBUG_PRINTLN("Inicializando tarjeta SD con bus SPI personalizado...");
    if (SD.begin(SD_CONFIG)) {
        DEBUG_PRINTLN("Tarjeta SD inicializada correctamente.");
        sd_disponible = true;
    } else {
        DEBUG_PRINTLN("ERROR: Fallo al inicializar la tarjeta SD. El vuelo continuara sin registro local.");
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

        // 1. Estructura de telemetría binaria
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

        // 2. Envío por LoRa condicionado por variable
        if (lora_habilitado) {
            // ResponseStatus rs = e220ttl.sendMessage(&t, sizeof(Telemetria));
            ResponseStatus rs = e220ttl.sendMessage("Test");
            DEBUG_PRINT("Transmision numero: ");
            DEBUG_PRINTLN(numeroTransmision);
            numeroTransmision++;

            #ifdef DEBUG_USB
            int numero = rs.code;
            DEBUG_PRINTLN(numero);
            if (numero != 1) {
                DEBUG_PRINT("Error al transmitir por LoRa: ");
                DEBUG_PRINTLN(rs.getResponseDescription());
            }
            #endif
        } else {
            DEBUG_PRINTLN("[LoRa DESACTIVADO] Omitiendo transmision de paquete.");
        }

        // 3. Registro en MicroSD
        if (sd_disponible) {
            String json = "{";
            json += "\"acc_x\":" + String(acc_x, 3) + ",";
            json += "\"acc_y\":" + String(acc_y, 3) + ",";
            json += "\"acc_z\":" + String(acc_z, 3) + ",";
            json += "\"pitch\":" + String(pitch, 2) + ",";
            json += "\"roll\":" + String(roll, 2) + ",";
            json += "\"yaw\":" + String(yaw, 2) + ",";
            json += "\"vel\":" + String(vel_horizontal, 2) + ",";       
            json += "\"vel_max\":" + String(vel_max, 2) + ",";           
            json += "\"latitud\":" + String(latitud, 6) + ",";           
            json += "\"longitud\":" + String(longitud, 6) + ",";         
            json += "\"altura\":" + String(altura_filtrada, 2) + ",";     
            json += "\"altura_max\":" + String(altura_max, 2) + ",";     
            json += "\"vel_vert\":" + String(vel_vertical, 2) + ",";     
            json += "\"temperatura\":" + String(temperatura, 2) + ",";   
            json += "\"estado\":" + String(estado_vuelo) + ",";          
            json += "\"flag\":" + String(flag_continuidad);              
            json += "}";

            DEBUG_PRINTLN(json);

            File logFile = SD.open("vuelo.json", FILE_WRITE);
            if (logFile) {
                logFile.println(json);
                logFile.close(); 
            }
        }

        ThisThread::sleep_for(std::chrono::milliseconds(1000));
    }
}