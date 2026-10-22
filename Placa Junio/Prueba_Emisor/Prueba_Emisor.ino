#include <Arduino.h>

// Asignación de pines de tu PCB
#define LORA_AUX_PIN A3 
#define LORA_RX_PIN  A4 // RX del Nano -> Conectar a TXD del E220
#define LORA_TX_PIN  A5 // TX del Nano -> Conectar a RXD del E220
#define LORA_M1_PIN  A6 
#define LORA_M0_PIN  A7 

// Canal deseado (23 = 873.125 MHz)
#define CANAL_LORA 1

UART loraSerial(digitalPinToPinName(LORA_TX_PIN), digitalPinToPinName(LORA_RX_PIN), NC, NC);

void cambiarModo(uint8_t m0, uint8_t m1) {
  digitalWrite(LORA_M0_PIN, m0);
  digitalWrite(LORA_M1_PIN, m1);
  delay(50);
  // Esperar a que el módulo termine de cambiar de modo (AUX en HIGH)
  while (digitalRead(LORA_AUX_PIN) == LOW) delay(5);
}

void configurarModulo(uint8_t canal) {
  Serial.println("Poniendo LoRa en Modo Configuracion (M0=1, M1=1)...");
  cambiarModo(HIGH, HIGH); // Modo 3

  // Trama oficial Ebyte C0 (Guardar permanente): ADD=0, 9600bps, 2.4kbps, Canal, Transparente, Sin cifrado
  uint8_t cfg[] = { 0xC0, 0x00, 0x08, 0x00, 0x00, 0x62, 0x00, canal, 0x03, 0x00, 0x00 };
  loraSerial.write(cfg, sizeof(cfg));
  delay(100);

  // Leer respuesta de confirmación
  while (loraSerial.available()) loraSerial.read();

  Serial.println("Configuracion grabada. Pasando a Modo Normal (M0=0, M1=0)...");
  cambiarModo(LOW, LOW); // Modo 0 (Normal)
  Serial.print("Emisor listo en Canal: ");
  Serial.println(canal);
}

uint32_t contador = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Se ha abierto el puerto serie");
  delay(1000);

  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  pinMode(LORA_AUX_PIN, INPUT);

  loraSerial.begin(9600);

  Serial.println("\n=== EMISOR NANO 33 BLE (CON CONTROL DE M0/M1) ===");
  configurarModulo(CANAL_LORA);
  Serial.println("Iniciando envios...\n");
}

void loop() {
  String mensaje = "COHETE_PING_" + String(contador++);
  
  Serial.print("Enviando: " + mensaje);
  loraSerial.println(mensaje);

  delay(15);
  int aux = digitalRead(LORA_AUX_PIN);
  Serial.println(aux == LOW ? " -> [RF EMITIDA OK]" : " -> [SIN RF]");

  delay(1000);
}