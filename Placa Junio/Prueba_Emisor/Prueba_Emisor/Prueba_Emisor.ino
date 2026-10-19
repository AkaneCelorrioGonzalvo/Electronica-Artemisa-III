#include <Arduino.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include "Arduino_BMI270_BMM150.h"
#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>
#include <SPI.h>
#include <SdFat.h> 
#include "LoRa_E220.h"
#include <rtos.h>

using namespace rtos;

#define SIMULAR_GPS_NO_MONTADO
#define DEBUG_USB

#ifdef DEBUG_USB
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// Pines de la PCB
const int PIN_PARACAIDAS = D7;       
const int PIN_SENSE_PARACAIDAS = A0; 
const int PIN_BUZZER = D13;          

arduino::MbedSPI SD_SPI(D9, D11, D10); 
const int PIN_SD_CS = D12; 
SdFat SD; 
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &SD_SPI)

// Pines verificados del LoRa en tu PCB
#define LORA_AUX_PIN A3 
#define LORA_RX_PIN  A4 
#define LORA_TX_PIN  A5 
#define LORA_M1_PIN  A6 
#define LORA_M0_PIN  A7 

UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);
LoRa_E220 e220ttl(&loraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);

TinyGPSPlus gps;

struct __attribute__((packed)) Telemetria {
    float acc_x, acc_y, acc_z;
    float pitch, roll, yaw;
    float vel, vel_max;
    float latitud, longitud;
    int16_t altura, altura_max;
    float vel_vert;
    int16_t temperatura;
    int16_t estado;
    int16_t flag;
};

Thread hilosSensores(osPriorityAboveNormal);    
Thread hiloGPS(osPriorityNormal);              
Thread hiloControlVuelo(osPriorityHigh);       
Thread hiloTelemetria(osPriorityBelowNormal);   

struct FiltroKalman1D {
    float q, r, x, p, k;
    void inicializar(float x_inicial, float ruido_proceso, float ruido_medida) {
        x = x_inicial; q = ruido_proceso; r = ruido_medida; p = 1.0;
    }
    float actualizar(float medida) {
        p += q; k = p / (p + r); x += k * (medida - x); p = (1.0 - k) * p;
        return x;
    }
} filtroAltura;

bool sd_disponible = false;
float acc_x = 0, acc_y = 0, acc_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
float mag_x = 0, mag_y = 0, mag_z = 0;
float pressure_kPa = 101.325, pressure_inicial = 101.325, temperatura = 25.0;
float pitch = 0, roll = 0, yaw = 0;
float latitud = 0, longitud = 0, vel_horizontal = 0, vel_max = 0;
float altura_filtrada = 0, altura_anterior = 0, altura_max = -9999.0, vel_vertical = 0, vel_vertical_filtrada = 0;
int estado_vuelo = 0, flag_continuidad = 0; 
uint32_t tiempo_anterior_calc_vel = 0, tiempo_disparo_paracaidas = 0;

float aleatorioFloat(float min, float max, int dec = 3) {
    float esc = pow(10, dec);
    return random((long)(min * esc), (long)(max * esc) + 1) / esc;
}

void taskSensores() {
    while (true) {
        if (IMU.accelerationAvailable()) IMU.readAcceleration(acc_x, acc_y, acc_z);
        if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        if (IMU.magneticFieldAvailable())IMU.readMagneticField(mag_x, mag_y, mag_z);

        pressure_kPa = BARO.readPressure();
        temperatura = HS300x.readTemperature();

        pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI;
        roll  = atan2(acc_y, acc_z) * 180.0 / M_PI;
        yaw   = atan2(mag_y, mag_x) * 180.0 / M_PI;
        if (yaw < 0) yaw += 360.0;

        ThisThread::sleep_for(std::chrono::milliseconds(20)); 
    }
}

void taskGPS() {
    while (true) {
#ifndef SIMULAR_GPS_NO_MONTADO
        while (Serial1.available() > 0) gps.encode(Serial1.read());
        ThisThread::sleep_for(std::chrono::milliseconds(2)); 
#else
        ThisThread::sleep_for(std::chrono::milliseconds(200));
#endif
    }
}

void taskControlVuelo() {
    tiempo_anterior_calc_vel = millis();
    while (true) {
        float alt_raw = 44330.0 * (1.0 - pow(pressure_kPa / pressure_inicial, 1.0 / 5.255));
        altura_filtrada = filtroAltura.actualizar(alt_raw);

        if (altura_max == -9999.0 || altura_filtrada > altura_max) altura_max = altura_filtrada;

        uint32_t t_act = millis();
        float dt = (t_act - tiempo_anterior_calc_vel) / 1000.0; 
        if (dt > 0.0) {
            float vel_inst = (altura_filtrada - altura_anterior) / dt;
            vel_vertical_filtrada = (0.75 * vel_vertical_filtrada) + (0.25 * vel_inst);
            vel_vertical = vel_vertical_filtrada;
        }
        altura_anterior = altura_filtrada;
        tiempo_anterior_calc_vel = t_act;

        flag_continuidad = (analogRead(PIN_SENSE_PARACAIDAS) > 500) ? 1 : 0;

        switch (estado_vuelo) {
            case 0: digitalWrite(PIN_PARACAIDAS, LOW); if (altura_filtrada > 10.0) estado_vuelo = 1; break;
            case 1: digitalWrite(PIN_PARACAIDAS, LOW); 
                    if (altura_filtrada > 15.0 && vel_vertical < -1.0) {
                        digitalWrite(PIN_PARACAIDAS, HIGH);
                        tiempo_disparo_paracaidas = millis();
                        estado_vuelo = 2;
                    }
                    break;
            case 2: if (millis() - tiempo_disparo_paracaidas >= 2000) digitalWrite(PIN_PARACAIDAS, LOW);
                    if (altura_filtrada < 3.0 && abs(vel_vertical) < 0.2) estado_vuelo = 3;
                    break;
            case 3: digitalWrite(PIN_PARACAIDAS, LOW); break;
        }

        digitalWrite(PIN_BUZZER, (estado_vuelo == 3 && (millis() / 500) % 2 == 0) ? HIGH : LOW);
        ThisThread::sleep_for(std::chrono::milliseconds(20));
    }
}

void taskTelemetria() {
    while (true) {
#ifdef SIMULAR_GPS_NO_MONTADO
        latitud  = aleatorioFloat(41.640000, 41.660000, 6);
        longitud = aleatorioFloat(-0.900000, -0.870000, 6);
        vel_horizontal = aleatorioFloat(0.0, 300.0, 2); 
#else
        latitud  = gps.location.isValid() ? gps.location.lat() : 0.0;
        longitud = gps.location.isValid() ? gps.location.lng() : 0.0;
        vel_horizontal = gps.speed.isValid() ? gps.speed.mps() : 0.0;
#endif
        if (vel_horizontal > vel_max) vel_max = vel_horizontal;

        Telemetria t;
        t.acc_x = acc_x; t.acc_y = acc_y; t.acc_z = acc_z;
        t.pitch = pitch; t.roll = roll; t.yaw = yaw;
        t.vel = vel_horizontal; t.vel_max = vel_max;
        t.latitud = latitud; t.longitud = longitud;
        t.altura = (int16_t)altura_filtrada;
        t.altura_max = (int16_t)altura_max;
        t.vel_vert = vel_vertical;
        t.temperatura = (int16_t)temperatura;
        t.estado = (int16_t)estado_vuelo;
        t.flag = (int16_t)flag_continuidad;

        // Envío por radio
        e220ttl.sendMessage(&t, sizeof(Telemetria));

        // Registro local en tarjeta SD
        if (sd_disponible) {
            String json = "{\"alt\":" + String(altura_filtrada, 1) + ",\"v_vert\":" + String(vel_vertical, 1) + "}";
            File logFile = SD.open("vuelo.json", FILE_WRITE);
            if (logFile) {
                logFile.println(json);
                logFile.close(); 
            }
        }

        // CADENCIA DE RADIO: 300 ms (~3.3 Hz) permite emitir sin desbordar el buffer del E220
        ThisThread::sleep_for(std::chrono::milliseconds(300));
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600); // GPS
    loraSerial.begin(9600);

    delay(1000);
    e220ttl.begin();

    // Sincronizar parámetros probados (Canal 23, Transparente, 2.4 kbps)
    ResponseStructContainer c = e220ttl.getConfiguration();
    if (c.status.code == 1) {
        Configuration cfg = *(Configuration*) c.data;
        cfg.CHAN = 23; // 873.125 MHz probado con éxito
        cfg.ADDH = 0x00;
        cfg.ADDL = 0x00;
        cfg.SPED.uartBaudRate = UART_BPS_9600;
        cfg.SPED.airDataRate  = AIR_DATA_RATE_010_24;
        cfg.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
        cfg.CRYPT.CRYPT_H = 0x00;
        cfg.CRYPT.CRYPT_L = 0x00;
        e220ttl.setConfiguration(cfg, WRITE_CFG_PWR_DWN_SAVE);
    }
    e220ttl.setMode(MODE_0_NORMAL);

    pinMode(PIN_PARACAIDAS, OUTPUT); digitalWrite(PIN_PARACAIDAS, LOW);
    pinMode(PIN_BUZZER, OUTPUT);     digitalWrite(PIN_BUZZER, LOW);
    pinMode(PIN_SENSE_PARACAIDAS, INPUT);

    IMU.begin();
    BARO.begin();
    HS300x.begin();

    // Calibración de presión base en rampa de lanzamiento
    float suma_p = 0;
    for (int i = 0; i < 30; i++) { suma_p += BARO.readPressure(); delay(20); }
    pressure_inicial = suma_p / 30.0;
    filtroAltura.inicializar(0.0, 0.05, 4.0);

    sd_disponible = SD.begin(SD_CONFIG);

    hilosSensores.start(taskSensores);
    hiloGPS.start(taskGPS);
    hiloControlVuelo.start(taskControlVuelo);
    hiloTelemetria.start(taskTelemetria);
}

void loop() {
    ThisThread::sleep_for(std::chrono::milliseconds(1000));
}