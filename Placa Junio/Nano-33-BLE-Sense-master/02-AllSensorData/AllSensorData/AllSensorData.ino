#include <Arduino.h>
#include <math.h>
#include <TinyGPSPlus.h> // Librería TinyGPSPlusPlus para decodificar GPS

// Librerías de los sensores integrados (Arduino Nano 33 BLE Sense Rev2)
#include "Arduino_BMI270_BMM150.h"
#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>

// ============================================================
// CONFIGURACIÓN DE PRUEBAS / SIMULACIÓN
// ============================================================
// Comenta la siguiente línea únicamente cuando tengas el GPS LC86L soldado en la PCB.
#define SIMULAR_GPS_NO_MONTADO

// ============================================================
// CONFIGURACIÓN DE PINES (PCBWAY)
// ============================================================
const int PIN_PARACAIDAS = D7;       // Salida digital para activar el MOSFET Q3 (Pólvora)
const int PIN_SENSE_PARACAIDAS = A0; // Entrada analógica para leer la línea SENSE1 (Continuidad)

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
// VARIABLES GLOBALES
// ============================================================

// Variables de sensores internos
float acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
float pressure_kPa = 101.325;
float pressure_inicial = 101.325; 
float temperatura = 25.0;

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
// 0 -> REPOSO, 1 -> ASCENSO, 2 -> DESCENSO, 3 -> ATERRIZAJE
int estado_vuelo = 0; 
int flag_continuidad = 0; // 1 -> Conectado, 0 -> Desconectado / Quemado

uint32_t tiempo_anterior_envio = 0;
uint32_t tiempo_anterior_calc_vel = 0;
uint32_t tiempo_disparo_paracaidas = 0;


// ============================================================
// INICIALIZACIÓN
// ============================================================

void inicializarComunicaciones()
{
    Serial.begin(BAUDRATE_DEBUG);     
    Serial1.begin(BAUDRATE_GPS);      
    loraSerial.begin(BAUDRATE_LORA);  

    // Cambiado a A1 para no interferir con la medición analógica de A0
    randomSeed(analogRead(A1)); 
}

void inicializarSensores()
{
    // Configuración de pines del sistema de eyección
    pinMode(PIN_PARACAIDAS, OUTPUT);
    digitalWrite(PIN_PARACAIDAS, LOW); // Seguridad absoluta en el encendido
    
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

    // Calibración de presión en rampa de lanzamiento
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

    // Inicializar el Filtro de Kalman de altura a 0.0 metros
    filtroAltura.inicializar(0.0, 0.05, 4.0);
}


// ============================================================
// LECTURAS Y ACTUALIZACIONES
// ============================================================

void actualizarSensores()
{
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
}

void actualizarGPS()
{
#ifndef SIMULAR_GPS_NO_MONTADO
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }
#endif
}


// ============================================================
// CONTROL DE CONTINUIDAD Y MÁQUINA DE ESTADOS DE VUELO
// ============================================================

void actualizarEstadoVuelo()
{
    // 1. Verificar la continuidad del inflamador en tiempo real
    // En el ADC de 10 bits de Arduino (0-1023), si está conectado (SENSE1 ≈ 3.3V) dará ~1023.
    // Si está desconectado o quemado (SENSE1 ≈ 0V por la R15 de 1M) dará ~0.
    int lectura_sense = analogRead(PIN_SENSE_PARACAIDAS);
    if (lectura_sense > 500) {
        flag_continuidad = 1; // Inflamador conectado y operativo
    } else {
        flag_continuidad = 0; // Inflamador ausente o quemado
    }

    // 2. Transiciones de la máquina de estados
    switch (estado_vuelo) {
        
        case 0: // REPOSO (En plataforma)
            digitalWrite(PIN_PARACAIDAS, LOW);
            // Si el cohete supera los 10 metros, declaramos inicio de ASCENSO
            if (altura_filtrada > 10.0) {
                estado_vuelo = 1;
            }
            break;

        case 1: // ASCENSO (Subiendo)
            digitalWrite(PIN_PARACAIDAS, LOW);
            // Detección de Apogeo: Altura mínima de seguridad (15m) para evitar activaciones en tierra
            // y velocidad vertical negativa persistente (empezó a caer)
            if (altura_filtrada > 15.0 && vel_vertical < -1.0) {
                digitalWrite(PIN_PARACAIDAS, HIGH); // !!! DISPARAR PARACAÍDAS !!!
                tiempo_disparo_paracaidas = millis();
                estado_vuelo = 2; // Transición a DESCENSO
            }
            break;

        case 2: // DESCENSO (Cayendo con paracaídas desplegado)
            // Lógica de seguridad para el MOSFET: Apagar el pulso tras 2 segundos (2000 ms)
            // Esto evita drenar la batería o quemar la resistencia de limitación de la PCB
            if (millis() - tiempo_disparo_paracaidas >= 2000) {
                digitalWrite(PIN_PARACAIDAS, LOW); 
            }

            // Detección de Aterrizaje: Altura inferior a 3m y velocidad casi nula
            if (altura_filtrada < 3.0 && abs(vel_vertical) < 0.2) {
                estado_vuelo = 3;
            }
            break;

        case 3: // ATERRIZAJE (En el suelo)
            digitalWrite(PIN_PARACAIDAS, LOW);
            break;

        default:
            digitalWrite(PIN_PARACAIDAS, LOW);
            estado_vuelo = 0;
            break;
    }
}


// ============================================================
// FUNCIONES AUXILIARES PARA DATOS SIMULADOS (GPS)
// ============================================================

float aleatorioFloat(float minimo, float maximo, int decimales = 3)
{
    float escala = pow(10, decimales);
    long min_int = (long)(minimo * escala);
    long max_int = (long)(maximo * escala);
    return random(min_int, max_int + 1) / escala;
}


// ============================================================
// PROCESAMIENTO Y ENVÍO DE TELEMETRÍA (JSON)
// ============================================================

void enviarDato()
{
    // 1. Orientación física real desde el IMU
    float pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI;
    float roll  = atan2(acc_y, acc_z) * 180.0 / M_PI;
    
    float yaw = atan2(mag_y, mag_x) * 180.0 / M_PI;
    if (yaw < 0) { yaw += 360.0; }

    // 2. Procesar datos del GPS (Real o Simulado)
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

    // 3. Altura relativa respecto al suelo (Launchpad)
    float altura_relativa_raw = 44330.0 * (1.0 - pow(pressure_kPa / pressure_inicial, 1.0 / 5.255));
    
    // Aplicar el Filtro de Kalman a la lectura cruda de la altura
    altura_filtrada = filtroAltura.actualizar(altura_relativa_raw);

    if (altura_max == -9999.0) {
        altura_max = altura_filtrada;
    } else if (altura_filtrada > altura_max) {
        altura_max = altura_filtrada;
    }

    // 4. Cálculo de Velocidad Vertical (derivada de la altura ya filtrada por Kalman)
    uint32_t tiempo_actual = millis();
    float dt = (tiempo_actual - tiempo_anterior_calc_vel) / 1000.0; 
    
    if (dt > 0.0) {
        float d_altura = altura_filtrada - altura_anterior; 
        float vel_vertical_inst = d_altura / dt; 

        // Suavizado ligero sobre la derivada
        vel_vertical_filtrada = (0.75 * vel_vertical_filtrada) + (0.25 * vel_vertical_inst);
        vel_vertical = vel_vertical_filtrada;
    }

    altura_anterior = altura_filtrada;
    tiempo_anterior_calc_vel = tiempo_actual;

    // 5. Construir el String JSON (Con datos de estado y flag REALES)
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
    json += "\"altura\":" + String(altura_filtrada, 2) + ",";     // Altura relativa al suelo filtrada
    json += "\"altura_max\":" + String(altura_max, 2) + ",";     // Altura máxima relativa filtrada
    json += "\"vel_vert\":" + String(vel_vertical, 2) + ",";     // Velocidad vertical estable
    json += "\"temperatura\":" + String(temperatura, 2) + ",";   
    json += "\"estado\":" + String(estado_vuelo) + ",";          // ESTADO REAL: 0->Reposo, 1->Ascenso, 2->Descenso, 3->Aterrizaje
    json += "\"flag\":" + String(flag_continuidad);              // CONTINUIDAD REAL: 1->Conectado, 0->Ausente/Quemado
    json += "}";

    // Enviar a la PC por USB
    Serial.println(json);

    // Enviar al módulo LoRa (físicamente por A4/A5)
    loraSerial.println(json);
}


// ============================================================
// ENVÍO PERIÓDICO A 10 Hz
// ============================================================

void ejecutarEnvioPeriodico()
{
    uint32_t tiempo_actual = millis();

    if (tiempo_actual - tiempo_anterior_envio >= INTERVALO_ENVIO_MS)
    {
        tiempo_anterior_envio = tiempo_actual;
        enviarDato();
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    inicializarComunicaciones();
    inicializarSensores();
    
    // Sincronizar el tiempo de cálculo inicial
    tiempo_anterior_calc_vel = millis();
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop()
{
    actualizarSensores();     // Lee acelerómetro, barómetro y temperatura
    actualizarGPS();          // Escucha el buffer del GPS (si no está simulado)
    actualizarEstadoVuelo();  // Procesa continuidad y la lógica de eyección de paracaídas (D7 y A0)
    ejecutarEnvioPeriodico(); // Genera y envía el JSON de telemetría
}