#include <Arduino.h>
#include <math.h>
#include <TinyGPSPlus.h> // Librería TinyGPSPlusPlus para decodificar GPS

// Librerías de los sensores integrados (Arduino Nano 33 BLE Sense Rev2)
#include "Arduino_BMI270_BMM150.h"
#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>

// Librerías para almacenamiento local en tarjeta MicroSD (SdFat)
#include <SPI.h>
#include <SdFat.h> 

// [LORA DESACTIVADO TEMPORALMENTE]
// #include "LoRa_E220.h"

// Librerías nativas de RTOS en Mbed OS
#include <rtos.h>

using namespace rtos;

// ============================================================
// CONFIGURACIÓN DE PRUEBAS / SIMULACIÓN / DEPURACIÓN
// ============================================================
#define SIMULAR_GPS_NO_MONTADO

// !!! ATENCIÓN !!! 
// Comenta la siguiente línea antes del lanzamiento para desactivar todo el 
// flujo de depuración por USB, optimizando la CPU de la placa en vuelo.
#define DEBUG_USB

// ------------------------------------------------------------
// CONFIGURACIÓN DE MACROS DE DEPURACIÓN CONDICIONAL (VARIÁDICAS)
// ------------------------------------------------------------
#ifdef DEBUG_USB
  #define DEBUG_BEGIN(baud)   Serial.begin(baud)
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ============================================================
// CONFIGURACIÓN DE PINES (PCB PERSONALIZADA)
// ============================================================
const int PIN_PARACAIDAS = D7;       
const int PIN_SENSE_PARACAIDAS = A0; 
const int PIN_BUZZER = D13;          
int numeroTransmision = 0;

// ------------------------------------------------------------
// ASIGNACIÓN DEL BUS SPI PERSONALIZADO PARA MICROSD (PCBWAY)
// ------------------------------------------------------------
arduino::MbedSPI SD_SPI(D9, D11, D10); 
const int PIN_SD_CS = D12; 

SdFat SD; 
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &SD_SPI)

// ------------------------------------------------------------
// ASIGNACIÓN DE PINES DEL LORA (E220) SEGÚN TU PCB
// ------------------------------------------------------------
// [LORA DESACTIVADO TEMPORALMENTE]
/*
#define LORA_AUX_PIN A3 
#define LORA_RX_PIN  A4 
#define LORA_TX_PIN  A5 
#define LORA_M1_PIN  A6 
#define LORA_M0_PIN  A7 

UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);
LoRa_E220 e220ttl(&loraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);
*/

// ============================================================
// CONFIGURACIÓN DE PUERTOS Y VELOCIDADES (BAUDRATES)
// ============================================================
const uint32_t BAUDRATE_DEBUG = 115200; 
const uint32_t BAUDRATE_GPS   = 9600;   

const uint32_t INTERVALO_ENVIO_MS = 100; // 100 ms = 10 Hz

TinyGPSPlus gps;

// ============================================================
// DEFINE ESTRUCTURA DE TELEMETRÍA (60 Bytes Totales)
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

// ============================================================
// INSTANCIACIÓN DE HILOS (THREADS / TASKS) DE RTOS
// ============================================================
Thread hilosSensores(osPriorityAboveNormal);    
Thread hiloGPS(osPriorityNormal);              
Thread hiloControlVuelo(osPriorityHigh);       
Thread hiloTelemetria(osPriorityBelowNormal);   

// ============================================================
// ESTRUCTURA DEL FILTRO DE KALMAN (1D para la Altura)
// ============================================================
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

FiltroKalman1D filtroAltura;

// ============================================================
// VARIABLES GLOBALES (Compartidas de forma segura)
// ============================================================
bool sd_disponible = true;

// Variables de sensores internos
float acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
float pressure_kPa = 101.325;
float pressure_inicial = 101.325; 
float temperatura = 25.0;

// Orientación calculada en tiempo real
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

// Variables de GPS y Velocidad Horizontal
float latitud = 0.0;
float longitud = 0.0;
float vel_horizontal = 0.0; 
float vel_max = 0.0;

// Variables de Altura y Velocidad Vertical
float altura_filtrada = 0.0;
float altura_anterior = 0.0;
float altura_max = -9999.0;
float vel_vertical = 0.0;
float vel_vertical_filtrada = 0.0;

// Máquina de estados de vuelo real
int estado_vuelo = 0; 
int flag_continuidad = 0; 

uint32_t tiempo_anterior_calc_vel = 0;
uint32_t tiempo_disparo_paracaidas = 0;

// ============================================================
// FUNCIONES AUXILIARES PARA DATOS SIMULADOS
// ============================================================
float aleatorioFloat(float minimo, float maximo, int decimales = 3){
    float escala = pow(10, decimales);
    long min_int = (long)(minimo * escala);
    long max_int = (long)(maximo * escala);
    return random(min_int, max_int + 1) / escala;
}

int generarEstado() { return random(0, 5); }
int generarFlag()   { return random(0, 2); }

// ============================================================
// TAREAS ASÍNCRONAS DE RTOS (THREADS / TASKS)
// ============================================================

void taskSensores(){
    while (true) {
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(acc_x, acc_y, acc_z);
        }
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        }
        if (IMU.magneticFieldAvailable()) {
            IMU.readMagneticField(mag_x, mag_y, mag_z);
        }

        pressure_kPa = BARO.readPressure();
        temperatura = HS300x.readTemperature();

        pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI;
        roll  = atan2(acc_y, acc_z) * 180.0 / M_PI;
        
        yaw = atan2(mag_y, mag_x) * 180.0 / M_PI;
        if (yaw < 0) { yaw += 360.0; }

        ThisThread::sleep_for(std::chrono::milliseconds(20)); 
    }
}

void taskGPS(){
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

void taskControlVuelo(){
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
        if (lectura_sense > 500) {
            flag_continuidad = 1; 
        } else {
            flag_continuidad = 0; 
        }

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

void taskTelemetria(){
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

        // 1. Empaquetar los datos en la estructura binaria
        /*
        // [LORA DESACTIVADO TEMPORALMENTE]

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
        t.altura = altura_filtrada;
        t.altura_max = altura_max;
        t.vel_vert = vel_vertical;
        t.temperatura = temperatura;
        t.estado = (int16_t)estado_vuelo;
        t.flag = (int16_t)flag_continuidad;
        
        // 2. Transmitir el struct binario completo (60 bytes)
        //ResponseStatus rs = e220ttl.sendMessage(&t, sizeof(Telemetria));
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
        */

        // 3. Escribir localmente en la SD (Se mantiene en formato JSON para facilitar su lectura en la PC)
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

            DEBUG_PRINTLN(json); // Solo imprime en USB de depuración

            File logFile = SD.open("vuelo.json", FILE_WRITE);
            if (logFile) {
                logFile.println(json);
                logFile.close(); 
            }
        }

        ThisThread::sleep_for(std::chrono::milliseconds(1000));
    }
}

// ============================================================
// FUNCIONES DE INICIALIZACIÓN
// ============================================================

void inicializarComunicaciones()
{
    DEBUG_BEGIN(BAUDRATE_DEBUG); 
    Serial1.begin(BAUDRATE_GPS);      

    // [LORA DESACTIVADO TEMPORALMENTE]
    /*
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
        // Forzar modo transparente (evita que la librería intente interpretar los primeros bytes del struct como dirección)
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
    */
}

void inicializarSensores(){
    pinMode(PIN_PARACAIDAS, OUTPUT);
    digitalWrite(PIN_PARACAIDAS, LOW); 
    
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); 
    
    pinMode(PIN_SENSE_PARACAIDAS, INPUT);

    if (!IMU.begin()) {
        DEBUG_PRINTLN("{\"error\":\"Failed to initialize IMU!\"}");
        while (1);
    }

    if (!BARO.begin()) {
        DEBUG_PRINTLN("{\"error\":\"Failed to initialize Pressure Sensor!\"}");
        while (1);
    }

    if (!HS300x.begin()) {
        DEBUG_PRINTLN("{\"error\":\"Failed to initialize Temperature/Humidity Sensor!\"}");
        while (1);
    }

    DEBUG_PRINTLN("Estabilizando y calibrando presion en tierra...");
    delay(1000); 
    float suma_presion = 0.0;
    const int MUESTRAS_CALIBRACION = 50;
    for (int i = 0; i < MUESTRAS_CALIBRACION; i++) {
        suma_presion += BARO.readPressure();
        delay(30); 
    }
    pressure_inicial = suma_presion / MUESTRAS_CALIBRACION;
    
    #ifdef DEBUG_USB
    DEBUG_PRINT("Presion de referencia (P0): ");
    DEBUG_PRINT(pressure_inicial, 3);
    DEBUG_PRINTLN(" kPa");
    #endif

    filtroAltura.inicializar(0.0, 0.05, 4.0);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    delay(2000);
    DEBUG_PRINTLN("Inicializando Comunicaciones");
    inicializarComunicaciones();

    DEBUG_PRINTLN("Inicializando tarjeta SD con bus SPI personalizado...");
    
    if (SD.begin(SD_CONFIG)) {
        DEBUG_PRINTLN("Tarjeta SD inicializada correctamente.");
        sd_disponible = true;
    } else {
        DEBUG_PRINTLN("ERROR: Fallo al inicializar la tarjeta SD. El vuelo continuara sin registro local.");
        sd_disponible = false;
    }

    inicializarSensores();
    
    tiempo_anterior_calc_vel = millis();

    randomSeed(analogRead(A1)); 

    hilosSensores.start(taskSensores);
    hiloGPS.start(taskGPS);
    hiloControlVuelo.start(taskControlVuelo);
    hiloTelemetria.start(taskTelemetria);
}

// ============================================================
// LOOP PRINCIPAL (MAIN THREAD)
// ============================================================

void loop()
{
    ThisThread::sleep_for(std::chrono::milliseconds(1000)); 
}