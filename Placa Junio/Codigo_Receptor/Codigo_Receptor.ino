#include <Arduino.h>
#include "LoRa_E220.h"

// Definición de pines para el ESP32-S3-Zero
#define RX_PIN 44 // Pin RX de la placa (conectar al TX del LoRa)
#define TX_PIN 43 // Pin TX de la placa (conectar al RX del LoRa)
#define AUX_PIN 6
#define M1_PIN 7
#define M0_PIN 8

// Inicializamos el objeto LoRa en el puerto Serial2 del ESP32-S3
LoRa_E220 e220ttl(&Serial2, AUX_PIN, M0_PIN, M1_PIN);


// ============================================================
// DEFINE ESTRUCTURA DE TELEMETRÍA (Exactamente idéntica a la del Emisor)
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
// SETUP
// ============================================================

void setup() {
  // Iniciamos la comunicación con el PC por USB a alta velocidad
  Serial.begin(115200);
  
  // El ESP32-S3 usa USB nativo, le damos un par de segundos para que 
  // el monitor serie del PC se conecte y no perder datos iniciales.
  delay(2000); 
  
  Serial.println("Configurando puerto serie para LoRa...");
  pinMode(RX_PIN, INPUT);
  
  // Iniciamos Serial2 explícitamente con los pines del ESP32-S3-Zero
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("Iniciando modulo LoRa E220...");
  e220ttl.begin();

  // Configuración de sincronización de canal
// Sincronización estricta de Canal, Dirección y Modo Transparente
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
    // Asegurar modo transparente para emparejarlo con el emisor
    if (config.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION) {
        config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
        cambiar_config = true;
    }

    if (cambiar_config) {
        Serial.println("Actualizando la configuracion del LoRa (Receptor)...");
        e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
        Serial.println("¡Configuracion del Receptor guardada con exito!");
    } else {
        Serial.println("La configuracion del LoRa ya es correcta en el Receptor.");
    }
  }

  e220ttl.setMode(MODE_0_NORMAL);
  Serial.println("Receptor listo y escuchando...");
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop() {
  // Si hay datos entrantes desde el cohete...
  if (e220ttl.available() > 0) {
    
    // Solicitamos leer una estructura del tamaño exacto del paquete de telemetría (60 bytes)
    ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(Telemetria));
    
    // Verificar si se leyó correctamente (code 1 = Éxito)
    if (rsc.status.code == 1) {
      // Reconstruimos el struct a partir de los bytes recibidos
      /*
      Telemetria t = *(Telemetria*) rsc.data;

      // Generamos el formato JSON idéntico para que el visualizador en Python lo entienda directamente
      String json = "{";
      json += "\"acc_x\":" + String(t.acc_x, 3) + ",";
      json += "\"acc_y\":" + String(t.acc_y, 3) + ",";
      json += "\"acc_z\":" + String(t.acc_z, 3) + ",";
      json += "\"pitch\":" + String(t.pitch, 2) + ",";
      json += "\"roll\":" + String(t.roll, 2) + ",";
      json += "\"yaw\":" + String(t.yaw, 2) + ",";
      json += "\"vel\":" + String(t.vel, 2) + ",";       
      json += "\"vel_max\":" + String(t.vel_max, 2) + ",";           
      json += "\"latitud\":" + String(t.latitud, 6) + ",";           
      json += "\"longitud\":" + String(t.longitud, 6) + ",";         
      json += "\"altura\":" + String(t.altura) + ",";     
      json += "\"altura_max\":" + String(t.altura_max) + ",";     
      json += "\"vel_vert\":" + String(t.vel_vert, 2) + ",";     
      json += "\"temperatura\":" + String(t.temperatura) + ",";   
      json += "\"estado\":" + String(t.estado) + ",";          
      json += "\"flag\":" + String(t.flag);              
      json += "}";

      // El print va directo al puerto serie del PC (que escucha el Python)
      Serial.println(json);*/
      String test = e220ttl.receiveMessage().data;
      Serial.println(test);

    } else {
      Serial.print("Error RX: ");
      Serial.println(rsc.status.getResponseDescription());
    }

    // Cerramos el contenedor para liberar los búferes internos del ESP32 (muy importante para evitar fugas de memoria)
    rsc.close();
  }
}