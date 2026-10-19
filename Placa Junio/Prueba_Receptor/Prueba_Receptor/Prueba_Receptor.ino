#include <Arduino.h>
#include "LoRa_E220.h"

#define RX_PIN 44 
#define TX_PIN 43 
#define AUX_PIN 6
#define M1_PIN  7
#define M0_PIN  8

// Constructor seguro que preserva el ruteo de pines UART del ESP32-S3
LoRa_E220 e220ttl(RX_PIN, TX_PIN, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

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

void setup() {
  Serial.begin(115200);
  delay(2000); 

  e220ttl.begin();

  // Aseguramos que el receptor coincida en Canal 23 y parámetros
  ResponseStructContainer c = e220ttl.getConfiguration();
  if (c.status.code == 1) {
    Configuration cfg = *(Configuration*) c.data;
    cfg.CHAN = 23;
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
  Serial.println("Estación de tierra lista. Recibiendo telemetría en tiempo real...");
}

void loop() {
  if (e220ttl.available() > 0) {
    // Lectura del paquete binario de 60 bytes
    ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(Telemetria));
    
    if (rsc.status.code == 1) {
      Telemetria t = *(Telemetria*) rsc.data;

      // Generación del JSON para el software de telemetría de tierra (Python/Ground Station)
      String json = "{";
      json += "\"acc_x\":" + String(t.acc_x, 2) + ",";
      json += "\"acc_y\":" + String(t.acc_y, 2) + ",";
      json += "\"acc_z\":" + String(t.acc_z, 2) + ",";
      json += "\"pitch\":" + String(t.pitch, 1) + ",";
      json += "\"roll\":" + String(t.roll, 1) + ",";
      json += "\"yaw\":" + String(t.yaw, 1) + ",";
      json += "\"vel\":" + String(t.vel, 1) + ",";       
      json += "\"vel_max\":" + String(t.vel_max, 1) + ",";           
      json += "\"lat\":" + String(t.latitud, 6) + ",";           
      json += "\"lon\":" + String(t.longitud, 6) + ",";         
      json += "\"alt\":" + String(t.altura) + ",";     
      json += "\"alt_max\":" + String(t.altura_max) + ",";     
      json += "\"vel_v\":" + String(t.vel_vert, 1) + ",";     
      json += "\"temp\":" + String(t.temperatura) + ",";   
      json += "\"estado\":" + String(t.estado) + ",";          
      json += "\"flag\":" + String(t.flag);              
      json += "}";

      Serial.println(json);
    }
    rsc.close();
  }
}