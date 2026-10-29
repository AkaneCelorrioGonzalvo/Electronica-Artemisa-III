#include <Arduino.h>
#include "LoRa_E220.h"
#include <string.h>
#include <math.h>

// Definición de pines para el ESP32-S3-Zero
#define RX_PIN 9  // Pin RX de la placa (conectar al TX del LoRa)
#define TX_PIN 10  // Pin TX de la placa (conectar al RX del LoRa)
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



// Protocolo v1: magic[4], version[1], longitud[1], secuencia LE[2],
// Telemetria[54], CRC16-CCITT-FALSE LE[2]. Total: 64 bytes.
// CRC sobre los primeros 62 bytes; init=0xFFFF, polinomio=0x1021.
static_assert(sizeof(float) == 4 && sizeof(Telemetria) == 54,
              "Formato de Telemetria incompatible");
static const size_t TAM_TRAMA = 64;
static const uint8_t CABECERA[4] = {0xA5, 0x5A, 0xC3, 0x3C};
static uint16_t crcTrama(const uint8_t *datos, size_t longitud) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < longitud; ++i) {
    crc ^= (uint16_t)datos[i] << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
  }
  return crc;
}
static void construirTrama(const Telemetria &t, uint16_t secuencia,
                          uint8_t *salida) {
  memcpy(salida, CABECERA, 4);
  salida[4] = 1;
  salida[5] = sizeof(Telemetria);
  salida[6] = (uint8_t)secuencia;
  salida[7] = (uint8_t)(secuencia >> 8);
  memcpy(salida + 8, &t, sizeof(t));
  uint16_t crc = crcTrama(salida, TAM_TRAMA - 2);
  salida[62] = (uint8_t)crc;
  salida[63] = (uint8_t)(crc >> 8);
}

static bool radioLista = false;

// ============================================================
// SETUP
// ============================================================
/*
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

    if (!e220ttl.begin()) {
        Serial.println("[LoRa] ERROR en begin; revisar AUX/modos/alimentacion.");
        return;
    }
    ResponseStructContainer c = e220ttl.getConfiguration();
    if (c.status.code != 1) {
        Serial.println("[LoRa] ERROR al leer configuracion:");
        Serial.println(c.status.getResponseDescription());
        // Algunas versiones dejan data sin inicializar si falla antes de leer.
        return;
    }
    if (c.data == nullptr) {
        Serial.println("[LoRa] Configuracion vacia.");
        return;
    }
    Configuration config = *(Configuration *)c.data;
    c.close();
    Serial.println("[LoRa] Velocidad de radio conservada:");
    Serial.println(config.SPED.getAirDataRateDescription());
    bool cambiar = config.CHAN != 1 || config.ADDH != 0 || config.ADDL != 0 ||
        config.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION ||
        config.TRANSMISSION_MODE.enableRSSI != 0;
    config.CHAN = 1;
    config.ADDH = 0;
    config.ADDL = 0;
    config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    config.TRANSMISSION_MODE.enableRSSI = 0;
    if (cambiar) {
        ResponseStatus resultado = e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
        if (resultado.code != 1) {
            Serial.println("[LoRa] ERROR al guardar configuracion:");
            Serial.println(resultado.getResponseDescription());
            return;
        }
    }
    ResponseStructContainer comprobacion = e220ttl.getConfiguration();
    if (comprobacion.status.code != 1) {
        Serial.println("[LoRa] ERROR al verificar configuracion:");
        Serial.println(comprobacion.status.getResponseDescription());
        return;
    }
    if (comprobacion.data == nullptr) return;
    Configuration verificada = *(Configuration *)comprobacion.data;
    comprobacion.close();
    if (verificada.CHAN != 1 || verificada.ADDH != 0 || verificada.ADDL != 0 ||
        verificada.TRANSMISSION_MODE.fixedTransmission != FT_TRANSPARENT_TRANSMISSION ||
        verificada.TRANSMISSION_MODE.enableRSSI != 0) {
        Serial.println("[LoRa] ERROR: configuracion no coincide.");
        return;
    }
    if (e220ttl.setMode(MODE_0_NORMAL) != 1) {
        Serial.println("[LoRa] ERROR al entrar en modo normal.");
        return;
    }
    radioLista = true;
    Serial.println("[LoRa] Inicializado: protocolo CRC v1, trama de 64 bytes.");

}
*/
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("=== TELEMETRIA REAL POR E220 ===");

  radioLista = false;

  // Pines del receptor que ya estaban funcionando.
  //Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); //Me ha dicho gemini que lo haga para evitar la fragmentación
  if (!e220ttl.begin()) {
    Serial.println("ERROR: no se ha podido iniciar el E220.");
    return;
  }

  ResponseStructContainer c = e220ttl.getConfiguration();

  if (c.status.code != 1) {
    Serial.print("ERROR leyendo configuracion: ");
    Serial.println(c.status.getResponseDescription());
    return;
  }

  if (c.data == nullptr) {
    Serial.println("ERROR: configuracion vacia.");
    return;
  }

  Configuration config = *(Configuration *)c.data;
  c.close();

  Serial.print("Velocidad de radio: ");
  Serial.println(config.SPED.getAirDataRateDescription());

  bool cambiar = //Habría que cambiar al canal 19, que el 1 no se puede usar. Y hay que tener un Duty de menos del 10%
    config.CHAN != 1 ||
    config.ADDH != 0 ||
    config.ADDL != 0 ||
    config.TRANSMISSION_MODE.fixedTransmission !=
      FT_TRANSPARENT_TRANSMISSION ||
    config.TRANSMISSION_MODE.enableRSSI != 0;

  config.CHAN = 1;
  config.ADDH = 0;
  config.ADDL = 0;
  config.TRANSMISSION_MODE.fixedTransmission =
    FT_TRANSPARENT_TRANSMISSION;
  config.TRANSMISSION_MODE.enableRSSI = 0;

  if (cambiar) {
    ResponseStatus resultado =
      e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);

    if (resultado.code != 1) {
      Serial.print("ERROR guardando configuracion: ");
      Serial.println(resultado.getResponseDescription());
      return;
    }
  }

  ResponseStructContainer comprobacion =
    e220ttl.getConfiguration();

  if (comprobacion.status.code != 1) {
    Serial.print("ERROR verificando configuracion: ");
    Serial.println(comprobacion.status.getResponseDescription());
    return;
  }

  if (comprobacion.data == nullptr) {
    Serial.println("ERROR: verificacion vacia.");
    return;
  }

  Configuration verificada =
    *(Configuration *)comprobacion.data;

  comprobacion.close();

  if (verificada.CHAN != 1 ||
      verificada.ADDH != 0 ||
      verificada.ADDL != 0 ||
      verificada.TRANSMISSION_MODE.fixedTransmission !=
        FT_TRANSPARENT_TRANSMISSION ||
      verificada.TRANSMISSION_MODE.enableRSSI != 0) {
    Serial.println("ERROR: configuracion no coincide.");
    return;
  }

  if (e220ttl.setMode(MODE_0_NORMAL) != 1) {
    Serial.println("ERROR: no se ha activado el modo normal.");
    return;
  }

  radioLista = true;

  Serial.println("Canal 1, direccion 0, modo transparente.");
  Serial.println("Esperando tramas de 64 bytes con CRC valido...");
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================


// Ventana deslizante: nunca vacia la UART al terminar una trama.
static uint8_t ventana[TAM_TRAMA];
static size_t usados = 0;
static uint32_t tramasOK = 0, erroresCRC = 0, descartados = 0, invalidos = 0;
static uint16_t ultimaSecuencia = 0;
static bool haySecuencia = false;
static uint32_t saltos = 0;

static void descartarPrimero() {
  memmove(ventana, ventana + 1, --usados);
  ++descartados;
}

static bool recibirByte(uint8_t dato, Telemetria &t, uint16_t &secuencia) {
  ventana[usados++] = dato;
  while (usados > 0) {
    size_t comparar = usados < 4 ? usados : 4;
    if (memcmp(ventana, CABECERA, comparar) != 0) {
      descartarPrimero();
      continue;
    }
    if (usados < 6) return false;
    if (ventana[4] != 1 || ventana[5] != sizeof(Telemetria)) {
      descartarPrimero();
      continue;
    }
    if (usados < TAM_TRAMA) return false;
    uint16_t recibido = (uint16_t)ventana[62] | ((uint16_t)ventana[63] << 8);
    if (recibido != crcTrama(ventana, 62)) {
      ++erroresCRC;
      descartarPrimero();
      continue;
    }
    memcpy(&t, ventana + 8, sizeof(t));
    secuencia = (uint16_t)ventana[6] | ((uint16_t)ventana[7] << 8);
    usados = 0;
    return true;
  }
  return false;
}
/*
void loop() {
  static uint8_t buffer[256];
  static size_t cantidad = 0;
  static unsigned long ultimoByte = 0;

  if (!radioLista) {
    delay(10);
    return;
  }

  // Acumular sin imprimir mientras entran los bytes.
  while (Serial2.available() > 0 &&
         cantidad < sizeof(buffer)) {
    int dato = Serial2.read();

    if (dato >= 0) {
      buffer[cantidad++] = (uint8_t)dato;
      ultimoByte = millis();
    }
  }

  // Volcar tras una pausa o cuando se llene el buffer.
  if (cantidad > 0 &&
      (millis() - ultimoByte >= 300 ||
       cantidad == sizeof(buffer))) {

    Serial.print("[RX HEX] bytes=");
    Serial.print(cantidad);
    Serial.print(" datos=");

    for (size_t i = 0; i < cantidad; i++) {
      if (buffer[i] < 0x10) Serial.print('0');
      Serial.print(buffer[i], HEX);
      Serial.print(' ');
    }

    Serial.println();
    cantidad = 0;
  }

  delay(1);
}*/
void loop() {
  static unsigned long ultimoResumen = 0;
  static unsigned long ultimaTelemetria = 0;
  static bool hayTelemetria = false;

  if (!radioLista) {
    if (millis() - ultimoResumen >= 5000) {
      ultimoResumen = millis();
      Serial.println(
        "ERROR: radio no inicializada. Revisar mensajes de arranque."
      );
    }

    delay(1);
    return;
  }

  while (Serial2.available() > 0) {
    int dato = Serial2.read();
    if (dato < 0) break;

    Telemetria t;
    uint16_t secuencia;

    // Solo devuelve true si la trama tiene cabecera,
    // longitud y CRC correctos.
    if (!recibirByte((uint8_t)dato, t, secuencia)) {
      continue;
    }

    ++tramasOK;

    if (haySecuencia &&
        secuencia != (uint16_t)(ultimaSecuencia + 1)) {
      ++saltos;
    }

    ultimaSecuencia = secuencia;
    haySecuencia = true;

    // Evitar mostrar datos no finitos o estados invalidos.
    if (!isfinite(t.acc_x) ||
        !isfinite(t.acc_y) ||
        !isfinite(t.acc_z) ||
        !isfinite(t.pitch) ||
        !isfinite(t.roll) ||
        !isfinite(t.yaw) ||
        !isfinite(t.vel) ||
        !isfinite(t.vel_max) ||
        !isfinite(t.latitud) ||
        !isfinite(t.longitud) ||
        !isfinite(t.vel_vert) ||
        t.estado < 0 || t.estado > 3 ||
        (t.flag != 0 && t.flag != 1)) {
      ++invalidos;
      continue;
    }

    ultimaTelemetria = millis();
    hayTelemetria = true;

    Serial.println();
    Serial.println("========== TELEMETRIA ==========");

    Serial.print("Secuencia: ");
    Serial.println(secuencia);

    Serial.print("Altura relativa: ");
    Serial.print(t.altura);
    Serial.println(" m");

    Serial.print("Altura maxima: ");
    Serial.print(t.altura_max);
    Serial.println(" m");

    Serial.print("Velocidad vertical: ");
    Serial.print(t.vel_vert, 2);
    Serial.println(" m/s");

    Serial.print("Velocidad horizontal GPS: ");
    Serial.print(t.vel, 2);
    Serial.println(" m/s");

    Serial.print("Velocidad horizontal maxima: ");
    Serial.print(t.vel_max, 2);
    Serial.println(" m/s");

    Serial.print("Temperatura: ");
    Serial.print(t.temperatura);
    Serial.println(" C");

    Serial.print("Latitud: ");
    Serial.println(t.latitud, 6);

    Serial.print("Longitud: ");
    Serial.println(t.longitud, 6);

    if (t.latitud == 0.0f && t.longitud == 0.0f) {
      Serial.println(
        "GPS: coordenadas cero; comprobar conexion y posicion valida."
      );
    }

    Serial.print("Aceleracion [g] X=");
    Serial.print(t.acc_x, 3);
    Serial.print(" Y=");
    Serial.print(t.acc_y, 3);
    Serial.print(" Z=");
    Serial.println(t.acc_z, 3);

    Serial.print("Orientacion [grados] Pitch=");
    Serial.print(t.pitch, 2);
    Serial.print(" Roll=");
    Serial.print(t.roll, 2);
    Serial.print(" Yaw=");
    Serial.println(t.yaw, 2);

    Serial.print("Estado de vuelo: ");
    Serial.println(t.estado);

    Serial.print("Flag de continuidad: ");
    Serial.println(t.flag);

    Serial.println("================================");
  }

  if (millis() - ultimoResumen >= 5000) {
    ultimoResumen = millis();

    Serial.println();
    Serial.print("# RX CRC_OK=");
    Serial.print(tramasOK);

    Serial.print(" CRC_ERROR=");
    Serial.print(erroresCRC);

    Serial.print(" BYTES_DESCARTADOS=");
    Serial.print(descartados);

    Serial.print(" DATOS_INVALIDOS=");
    Serial.print(invalidos);

    Serial.print(" SALTOS_SECUENCIA=");
    Serial.print(saltos);

    Serial.print(" PENDIENTES=");
    Serial.println(usados);

    if (!hayTelemetria) {
      Serial.println(
        "# Todavia no se ha recibido telemetria valida."
      );
    } else {
      unsigned long antiguedad = millis() - ultimaTelemetria;

      Serial.print("# Tiempo desde la ultima telemetria valida: ");
      Serial.print(antiguedad / 1000.0f, 1);
      Serial.println(" s");

      if (antiguedad >= 5000) {
        Serial.println(
          "# AVISO: sin datos recientes; las lecturas anteriores estan desactualizadas."
        );
      }
    }
  }

  delay(1);
}
