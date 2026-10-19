#include <Arduino.h>

#define LORA_AUX_PIN A3 
#define LORA_RX_PIN  A4 
#define LORA_TX_PIN  A5 

UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  loraSerial.begin(9600);
  pinMode(LORA_AUX_PIN, INPUT);

  Serial.println("\n=== NANO 33 BLE (MODO RECEPTOR) ===");
  Serial.println("A la espera de ráfagas desde el ESP32...");
}

void loop() {
  // 1. Si la antena capta radio, el pin AUX del Nano caerá a nivel bajo
  static int ultimoAux = HIGH;
  int auxActual = digitalRead(LORA_AUX_PIN);
  if (auxActual != ultimoAux) {
    if (auxActual == LOW) {
      Serial.println("\n[RF CAPTADA] Pin AUX del Nano en LOW: Onda detectada en la antena!");
    }
    ultimoAux = auxActual;
  }

  // 2. Si entran bytes por A4, imprimirlos
  if (loraSerial.available()) {
    Serial.print(">>> DATO RECIBIDO: ");
    while (loraSerial.available()) {
      char c = loraSerial.read();
      Serial.print(c);
    }
  }
}