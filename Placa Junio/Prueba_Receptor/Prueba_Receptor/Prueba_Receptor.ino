#include <Arduino.h>

#define RX_PIN  44 
#define TX_PIN  43 
#define AUX_PIN 6

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(AUX_PIN, INPUT);

  Serial.println("\n=== ESP32-S3 (MODO EMISOR) ===");
}

uint32_t c = 0;

void loop() {
  String msg = "PING_ESP_" + String(c++);
  Serial.print("Enviando: " + msg);

  // Enviamos por el puerto serie al LoRa
  Serial2.println(msg);

  // Comprobamos si el módulo LoRa del ESP32 responde bajando AUX
  delay(15); 
  int estadoAux = digitalRead(AUX_PIN);
  Serial.print(" | AUX del LoRa: ");
  Serial.println(estadoAux == LOW ? "0 (EMITIENDO RF)" : "1 (SIN EMISION)");

  delay(1500);
}