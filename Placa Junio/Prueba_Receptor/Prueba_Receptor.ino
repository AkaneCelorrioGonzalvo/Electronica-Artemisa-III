#include <Arduino.h>

#define RX_PIN  9 // RX del ESP32 -> Conectar a TXD del E220
#define TX_PIN  10 // TX del ESP32 -> Conectar a RXD del E220
#define AUX_PIN 6
#define M1_PIN  7
#define M0_PIN  8

// Mismo canal que el Emisor
#define CANAL_LORA 1

void cambiarModo(uint8_t m0, uint8_t m1) {
  digitalWrite(M0_PIN, m0);
  digitalWrite(M1_PIN, m1);
  delay(50);
  while (digitalRead(AUX_PIN) == LOW) delay(5);
}

void configurarModulo(uint8_t canal) {
  Serial.println("Poniendo LoRa en Modo Configuracion (M0=1, M1=1)...");
  cambiarModo(HIGH, HIGH); // Modo 3

  uint8_t cfg[] = { 0xC0, 0x00, 0x08, 0x00, 0x00, 0x62, 0x00, canal, 0x03, 0x00, 0x00 };
  Serial2.write(cfg, sizeof(cfg));
  delay(100);

  while (Serial2.available()) Serial2.read();

  Serial.println("Configuracion grabada. Pasando a Modo Normal (M0=0, M1=0)...");
  cambiarModo(LOW, LOW); // Modo 0 (Normal)
  Serial.print("Receptor listo en Canal: ");
  Serial.println(canal);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Se ha abierto el puerto serie");
  delay(1000);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);

  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("\n=== RECEPTOR ESP32-S3 (CON CONTROL DE M0/M1) ===");
  configurarModulo(CANAL_LORA);
  Serial.println("Escuchando radio...\n");
}

void loop() {
  // Testigo de recepción en antena por el pin AUX
  static int ultimoAux = HIGH;
  int auxActual = digitalRead(AUX_PIN);
  if (auxActual != ultimoAux) {
    if (auxActual == LOW) {
      Serial.println("\n[RF DETECTADA EN ANTENA]");
    }
    ultimoAux = auxActual;
  }

  // Lectura directa del mensaje entrante
  if (Serial2.available() > 0) {
    Serial.print(">>> RECIBIDO: ");
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      Serial.print(c);
    }
  }
}