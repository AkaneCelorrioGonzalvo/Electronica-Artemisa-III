#include <Arduino.h>
#include <math.h>
#include <TinyGPSPlus.h> // Librería TinyGPSPlusPlus para decodificar GPS

// Librerías de los sensores integrados (Arduino Nano 33 BLE Sense Rev2)
#include "Arduino_BMI270_BMM150.h"
#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>

// Librerías para almacenamiento local en tarjeta MicroSD (Migrado a SdFat)
#include <SPI.h>
#include <SdFat.h> // REEMPLAZADO: Usamos SdFat en lugar de SD.h

// Librerías nativas de RTOS en Mbed OS
#include <rtos.h>

using namespace rtos;

// ============================================================
// CONFIGURACIÓN DE PRUEBAS / SIMULACIÓN
// ============================================================
#define SIMULAR_GPS_NO_MONTADO

// ============================================================
// CONFIGURACIÓN DE PINES (PCB PERSONALIZADA)
// ============================================================
const int PIN_PARACAIDAS = D7;       
const int PIN_SENSE_PARACAIDAS = A0; 
const int PIN_BUZZER = D13;          

// ------------------------------------------------------------
// ASIGNACIÓN DEL BUS SPI PERSONALIZADO PARA MICROSD (PCBWAY)
// ------------------------------------------------------------
// MISO = D9, SCK = D10, MOSI = D11, CS = D12
// El constructor de MbedSPI toma los parámetros en el orden: (MISO, MOSI, SCK)
arduino::MbedSPI SD_SPI(D9, D11, D10); 
const int PIN_SD_CS = D12; 

// Instanciamos el objeto 'SD' usando la clase SdFat.
// Al llamarse 'SD', la compatibilidad con los métodos .open() y File se mantiene intacta.
SdFat SD; 

// Definimos la configuración para inicializar el bus SPI dedicado para la SD
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &SD_SPI)

// ============================================================
// CONFIGURACIÓN DE PUERTOS Y VELOCIDADES (BAUDRATES)
// ============================================================
const uint32_t BAUDRATE_DEBUG = 115200; 
const uint32_t BAUDRATE_GPS   = 9600;   
const uint32_t BAUDRATE_LORA  = 9600;   

const uint32_t INTERVALO_ENVIO_MS = 100; // 100 ms = 10 Hz

// Configuración de puertos serie de la placa
#define LORA_TX_PIN A5 
#define LORA_RX_PIN A4 
UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);

TinyGPSPlus gps;


// ============================================================
// INSTANCIACIÓN DE HILOS (THREADS / TASKS) DE RTOS
// ============================================================
Thread hilosSensores(osPriorityAboveNormal);    // Muestreo rápido de sensores
Thread hiloGPS(osPriorityNormal);              // Procesamiento de tramas GPS NMEA
Thread hiloControlVuelo(osPriorityHigh);       // Lógica de seguridad, disparo y zumbador
Thread hiloTelemetria(osPriorityBelowNormal);   // Salida serie lenta (LoRa) y guardado en SD


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

bool sd_disponible = false;

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

float aleatorioFloat(float minimo, float maximo, int decimales = 3)
{
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

// TAREA 1: Lectura y procesamiento de sensores internos (50 Hz)
void taskSensores()
{
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

        // Corregido: Se utiliza std::chrono::milliseconds para eliminar la advertencia de depreciación
        ThisThread::sleep_for(std::chrono::milliseconds(20)); 
    }
}

// TAREA 2: Escucha del buffer GPS (Ejecución casi continua)
void taskGPS()
{
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

// TAREA 3: Máquina de estados de vuelo, control de eyección y zumbador (50 Hz)
void taskControlVuelo()
{
    tiempo_anterior_calc_vel = millis();

    while (true) {
        // 1. Altura relativa respecto al suelo (Launchpad)
        float altura_relativa_raw = 44330.0 * (1.0 - pow(pressure_kPa / pressure_inicial, 1.0 / 5.255));
        
        // Aplicar el Filtro de Kalman
        altura_filtrada = filtroAltura.actualizar(altura_relativa_raw);

        if (altura_max == -9999.0) {
            altura_max = altura_filtrada;
        } else if (altura_filtrada > altura_max) {
            altura_max = altura_filtrada;
        }

        // 2. Cálculo de Velocidad Vertical
        uint32_t tiempo_actual = millis();
        float dt = (tiempo_actual - tiempo_anterior_calc_vel) / 1000.0; 
        
        if (dt > 0.0) {
            float d_altura = altura_filtrada - altura_anterior; 
            float vel_vertical_inst = d_altura / dt; 

            // Suavizado de la derivada
            vel_vertical_filtrada = (0.75 * vel_vertical_filtrada) + (0.25 * vel_vertical_inst);
            vel_vertical = vel_vertical_filtrada;
        }

        altura_anterior = altura_filtrada;
        tiempo_anterior_calc_vel = tiempo_actual;

        // 3. Procesar Continuidad del inflamador (Pin A0)
        int lectura_sense = analogRead(PIN_SENSE_PARACAIDAS);
        if (lectura_sense > 500) {
            flag_continuidad = 1; 
        } else {
            flag_continuidad = 0; 
        }

        // 4. Máquina de Estados de Vuelo
        switch (estado_vuelo) {
            case 0: // REPOSO (En plataforma)
                digitalWrite(PIN_PARACAIDAS, LOW);
                if (altura_filtrada > 10.0) {
                    estado_vuelo = 1;
                }
                break;

            case 1: // ASCENSO (Subiendo)
                digitalWrite(PIN_PARACAIDAS, LOW);
                if (altura_filtrada > 15.0 && vel_vertical < -1.0) {
                    digitalWrite(PIN_PARACAIDAS, HIGH); // !!! DISPARO MOSFET !!!
                    tiempo_disparo_paracaidas = millis();
                    estado_vuelo = 2; 
                }
                break;

            case 2: // DESCENSO
                // Seguridad del MOSFET: Cortar pulso a los 2 segundos
                if (millis() - tiempo_disparo_paracaidas >= 2000) {
                    digitalWrite(PIN_PARACAIDAS, LOW); 
                }
                if (altura_filtrada < 3.0 && abs(vel_vertical) < 0.2) {
                    estado_vuelo = 3;
                }
                break;

            case 3: // ATERRIZAJE
                digitalWrite(PIN_PARACAIDAS, LOW);
                break;
        }

        // 5. Control del Zumbador Localizador (Solo en estado de Aterrizaje)
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

// TAREA 4: Emisión de Telemetría JSON (10 Hz) y Almacenamiento en Tarjeta SD
void taskTelemetria()
{
    while (true) {
        // 1. Cargar GPS (físico o simulado)
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

        // 2. Construir el JSON de telemetría consolidado
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

        // 3. Envío de datos a los puertos serie (USB y LoRa)
        Serial.println(json);
        loraSerial.println(json);

        // 4. Guardar datos en la MicroSD usando el bus SPI personalizado 'SD_SPI' de SdFat
        if (sd_disponible) {
            File logFile = SD.open("vuelo.txt", FILE_WRITE);
            if (logFile) {
                logFile.println(json);
                logFile.close(); 
            }
        }

        ThisThread::sleep_for(std::chrono::milliseconds(100));
    }
}


// ============================================================
// FUNCIONES DE INICIALIZACIÓN
// ============================================================

void inicializarComunicaciones()
{
    Serial.begin(BAUDRATE_DEBUG);     
    Serial1.begin(BAUDRATE_GPS);      
    loraSerial.begin(BAUDRATE_LORA);  
}

void inicializarSensores()
{
    pinMode(PIN_PARACAIDAS, OUTPUT);
    digitalWrite(PIN_PARACAIDAS, LOW); 
    
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); 
    
    pinMode(PIN_SENSE_PARACAIDAS, INPUT);

    if (!IMU.begin()) {
        Serial.println("{\"error\":\"Failed to initialize IMU!\"}");
        while (1);
    }

    if (!BARO.begin()) {
        Serial.println("{\"error\":\"Failed to initialize Pressure Sensor!\"}");
        while (1);
    }

    if (!HS300x.begin()) {
        Serial.println("{\"error\":\"Failed to initialize Temperature/Humidity Sensor!\"}");
        while (1);
    }

    // Calibración inicial de la rampa de lanzamiento (P0)
    Serial.println("Estabilizando y calibrando presion en tierra...");
    delay(1000); 
    float suma_presion = 0.0;
    const int MUESTRAS_CALIBRACION = 50;
    for (int i = 0; i < MUESTRAS_CALIBRACION; i++) {
        suma_presion += BARO.readPressure();
        delay(30); 
    }
    pressure_inicial = suma_presion / MUESTRAS_CALIBRACION;
    Serial.print("Presion de referencia (P0): ");
    Serial.print(pressure_inicial, 3);
    Serial.println(" kPa");

    filtroAltura.inicializar(0.0, 0.05, 4.0);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    inicializarComunicaciones();

    // ------------------------------------------------------------
    // INICIALIZACIÓN DE LA TARJETA SD (BUS SPI PERSONALIZADO)
    // ------------------------------------------------------------
    Serial.println("Inicializando tarjeta SD con bus SPI personalizado...");
    
    // Corregido: Se utiliza la estructura SD_CONFIG de SdFat para heredar el bus SPI alternativo
    if (SD.begin(SD_CONFIG)) {
        Serial.println("Tarjeta SD inicializada correctamente.");
        sd_disponible = true;

        File logFile = SD.open("vuelo.txt", FILE_WRITE);
        if (logFile) {
            logFile.println("\n--- INICIO DE REGISTRO DE VUELO ---");
            logFile.close();
        }
    } else {
        Serial.println("ERROR: Fallo al inicializar la tarjeta SD. El vuelo continuara sin registro local.");
        sd_disponible = false;
    }

    inicializarSensores();
    
    tiempo_anterior_calc_vel = millis();

    randomSeed(analogRead(A1)); 

    // ------------------------------------------------------------
    // INICIAR HILOS DE RTOS (Tasks)
    // ------------------------------------------------------------
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