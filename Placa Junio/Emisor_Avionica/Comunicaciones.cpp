#include "Comunicaciones.h"
#include <string.h>
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
// CRC sobre los primeros 62 bytes; init=0xFFFF, polinomio=0x1021.
static_assert(sizeof(float) == 4 && sizeof(Telemetria) == 54,
              "Formato de Telemetria incompatible");
static const size_t TAM_TRAMA = 64;
static const uint8_t CABECERA[4] = {0xA5, 0x5A, 0xC3, 0x3C};
static uint16_t crcTrama(const uint8_t *datos, size_t longitud) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < longitud; ++i) {
    crc ^= (uint16_t)datos[i] << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
  }
  return crc;
}
static void construirTrama(const Telemetria &t, uint16_t secuencia,
                          uint8_t *salida) {
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

// SPI personalizado y SD
static arduino::MbedSPI SD_SPI(D9, D11, D10); 
static SdFat SD; 
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &SD_SPI)

// Instancia LoRa
static UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);
static LoRa_E220 e220ttl(&loraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);

static ResponseStatus enviarUARTByteAByte(
    const uint8_t *datos, size_t longitud) {

    ResponseStatus resultado;
    resultado.code = E220_SUCCESS;

    // Esperar a que el modulo este disponible.
    unsigned long inicio = millis();

    while (digitalRead(LORA_AUX_PIN) == LOW) {
        if (millis() - inicio >= 5000) {
            resultado.code = ERR_E220_TIMEOUT;
            return resultado;
        }

        ThisThread::sleep_for(std::chrono::milliseconds(1));
    }

    // Evitar la escritura de un bloque completo de ArduinoCore-mbed.
    // No introducir pausas voluntarias entre bytes.
    for (size_t i = 0; i < longitud; ++i) {
        if (loraSerial.write((uint8_t)datos[i]) != 1) {
            resultado.code = ERR_E220_DATA_SIZE_NOT_MATCH;
            return resultado;
        }
    }

    loraSerial.flush();

    // Dar tiempo a que AUX refleje el procesamiento del modulo.
    ThisThread::sleep_for(std::chrono::milliseconds(5));

    inicio = millis();

    while (digitalRead(LORA_AUX_PIN) == LOW) {
        if (millis() - inicio >= 5000) {
            resultado.code = ERR_E220_TIMEOUT;
            return resultado;
        }

        ThisThread::sleep_for(std::chrono::milliseconds(1));
    }

    return resultado;
}

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
    D_BEGIN(BAUDRATE_DEBUG); 
    Serial1.begin(BAUDRATE_GPS);      

    if (!lora_habilitado) {
        D_PRINTLN("[INFO] LoRa DESACTIVADO (lora_habilitado = false). Omitiendo inicio del hardware.");
        return;
    }

    //loraSerial.begin(9600); 
    loraSerial.begin(115200);//Me ha dicho Gemini que lo cambie para mitigar la fragmentación
    D_PRINTLN("Iniciando modulo LoRa E220...");

    if (!e220ttl.begin()) {
        D_PRINTLN("[LoRa] ERROR en begin; revisar AUX/modos/alimentacion.");
        return;
    }
    ResponseStructContainer c = e220ttl.getConfiguration();
    if (c.status.code != 1) {
        D_PRINTLN("[LoRa] ERROR al leer configuracion:");
        D_PRINTLN(c.status.getResponseDescription());
        // Algunas versiones dejan data sin inicializar si falla antes de leer.
        return;
    }
    if (c.data == nullptr) {
        D_PRINTLN("[LoRa] Configuracion vacia.");
        return;
    }
    Configuration config = *(Configuration *)c.data;
    c.close();
    D_PRINTLN("[LoRa] Velocidad de radio conservada:");
    D_PRINTLN(config.SPED.getAirDataRateDescription());
    bool cambiar = config.CHAN != 1 || config.ADDH != 0 || config.ADDL != 0 ||//Habría que cambiar al canal 19, que el 1 no se puede usar
                                                                              //Y hay que tener un Duty de menos del 10%
        config.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION ||
        config.TRANSMISSION_MODE.enableRSSI != 0;
    config.CHAN = 1;
    config.ADDH = 0;
    config.ADDL = 0;
    config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    config.TRANSMISSION_MODE.enableRSSI = 0;
    if (cambiar) {
        ResponseStatus resultado = e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
        if (resultado.code != 1) {
            D_PRINTLN("[LoRa] ERROR al guardar configuracion:");
            D_PRINTLN(resultado.getResponseDescription());
            return;
        }
    }
    ResponseStructContainer comprobacion = e220ttl.getConfiguration();
    if (comprobacion.status.code != 1) {
        D_PRINTLN("[LoRa] ERROR al verificar configuracion:");
        D_PRINTLN(comprobacion.status.getResponseDescription());
        return;
    }
    if (comprobacion.data == nullptr) return;
    Configuration verificada = *(Configuration *)comprobacion.data;
    comprobacion.close();
    if (verificada.CHAN != 1 || verificada.ADDH != 0 || verificada.ADDL != 0 ||
        verificada.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION ||
        verificada.TRANSMISSION_MODE.enableRSSI != 0) {
        D_PRINTLN("[LoRa] ERROR: configuracion no coincide.");
        return;
    }
    if (e220ttl.setMode(MODE_0_NORMAL) != 1) {
        D_PRINTLN("[LoRa] ERROR al entrar en modo normal.");
        return;
    }
    radioLista = true;
    D_PRINTLN("[LoRa] Inicializado: protocolo CRC v1, trama de 64 bytes.");

}

void inicializarSD() {
    D_PRINTLN("Inicializando tarjeta SD con bus SPI personalizado...");
    if (SD.begin(SD_CONFIG)) {
        D_PRINTLN("Tarjeta SD inicializada correctamente.");
        sd_disponible = true;
    } else {
        D_PRINTLN("ERROR: Fallo al inicializar la tarjeta SD. El vuelo continuara sin registro local.");
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
        if (lora_habilitado && radioLista) {
            uint8_t trama[TAM_TRAMA];
            construirTrama(t, secuenciaTX++, trama);
            ResponseStatus rs = enviarUARTByteAByte(trama, sizeof(trama));
            D_PRINT("[TX HEX] ");
            for (size_t i = 0; i < sizeof(trama); i++) {
            if (trama[i] < 0x10) D_PRINT('0');
            D_PRINT(trama[i], HEX);
            D_PRINT(' ');
            }
            D_PRINTLN();
            /*
            D_PRINT("[TX] secuencia=");
            D_PRINT((uint16_t)(secuenciaTX - 1));

            D_PRINT(" bytes=");
            D_PRINT(sizeof(trama));

            D_PRINT(" estado=");
            D_PRINT(rs.code);

            D_PRINT(" descripcion=");
            D_PRINTLN(rs.getResponseDescription());
            */
            //ResponseStatus rs = e220ttl.sendMessage("Test");
            D_PRINT("Transmision numero: ");
            D_PRINTLN(numeroTransmision);
            numeroTransmision++;

            #ifdef DEBUG_USB
            int numero = rs.code;
            D_PRINTLN(numero);
            if (numero != 1) {
                D_PRINT("Error al transmitir por LoRa: ");
                D_PRINTLN(rs.getResponseDescription());
            }
            #endif
        } else {
            //D_PRINTLN("[LoRa DESACTIVADO] Omitiendo transmision de paquete.");
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

            D_PRINTLN(json);
            File logFile = SD.open("vuelo.json", FILE_WRITE);
            if (logFile) {
                logFile.println(json);
                logFile.close(); 
            }
        }

        ThisThread::sleep_for(std::chrono::milliseconds(1000));
    }
}