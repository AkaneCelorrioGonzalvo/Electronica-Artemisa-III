#include "Sensores.h"
#include "Config.h"
#include "Globals.h"

#include <math.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>
#include <rtos.h>

using namespace rtos;

#define BMI270_I2C_ADDR   0x68
#define BMI270_ACC_RANGE  0x41
#define BMI270_RANGE_16G  0x03  // Desbloqueo a escala completa +/-16G

void inicializarSensores() {
    if (!IMU.begin()) {
        D_PRINTLN("{\"error\":\"Failed to initialize IMU!\"}");
        while (1);
    }

    // Configuración de escala a ±16G por Wire1 (bus interno del Nano 33 BLE)
    Wire1.beginTransmission(BMI270_I2C_ADDR);
    Wire1.write(BMI270_ACC_RANGE);
    Wire1.write(BMI270_RANGE_16G);
    if (Wire1.endTransmission() == 0) {
        D_PRINTLN("IMU configurada con exito a +/-16G.");
    } else {
        D_PRINTLN("{\"warning\":\"Fallo al poner rango a 16G, permanece en 4G\"}");
    }

    if (!BARO.begin()) {
        D_PRINTLN("{\"error\":\"Failed to initialize Pressure Sensor!\"}");
        while (1);
    }

    if (!HS300x.begin()) {
        D_PRINTLN("{\"error\":\"Failed to initialize Temperature/Humidity Sensor!\"}");
        while (1);
    }

    D_PRINTLN("Calibrando presion en tierra...");
    delay(1000); 
    float suma_presion = 0.0;
    const int MUESTRAS_CALIBRACION = 50;
    for (int i = 0; i < MUESTRAS_CALIBRACION; i++) {
        suma_presion += BARO.readPressure();
        delay(30); 
    }
    pressure_inicial = suma_presion / MUESTRAS_CALIBRACION;
    
    #ifdef DEBUG_USB
    D_PRINT("Presion de referencia (P0): ");
    D_PRINT(pressure_inicial, 3);
    D_PRINTLN(" kPa");
    #endif

    // q subido a 0.8 para eliminar el retraso (lag) durante la subida vertical rápida
    filtroAltura.inicializar(0.0, 0.8, 4.0);
}

void taskSensores() {
    while (true) {
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(acc_x, acc_y, acc_z);
            
            // Corrección de escala por 4 (la librería divide entre 8192 asumiendo 4G)
            acc_x *= 4.0f;
            acc_y *= 4.0f;
            acc_z *= 4.0f;
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