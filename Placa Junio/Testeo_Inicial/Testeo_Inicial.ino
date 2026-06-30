#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>

// Variable para almacenar la presión a nivel del suelo (calibración inicial)
float presionBase = 101.325; // Valor por defecto en kPa

void setup() {
  Serial.begin(115200);
  while (!Serial); // Espera a que se abra el Monitor Serie

  Serial.println("--- PRUEBA DE SENSORES NANO 33 BLE SENSE REV 2 ---");

  // 1. Inicializar IMU (Acelerómetro + Giroscopio + Magnetómetro)
  if (!IMU.begin()) {
    Serial.println("Error: No se pudo inicializar la IMU (BMI270/BMM150).");
    while (1);
  }
  Serial.println("IMU... OK");

  // 2. Inicializar Barómetro
  if (!BARO.begin()) {
    Serial.println("Error: No se pudo inicializar el barómetro (LPS22HB).");
    while (1);
  }
  Serial.println("Barómetro... OK");

  // 3. Inicializar Sensor de Temperatura y Humedad
  if (!HS300x.begin()) {
    Serial.println("Error: No se pudo inicializar el sensor HS300x.");
    while (1);
  }
  Serial.println("Temp/Humedad... OK");

  // Calibración rápida del nivel del suelo
  Serial.println("\nCalibrando presión base (no muevas el Arduino)...");
  float sumaPresion = 0;
  for (int i = 0; i < 10; i++) {
    sumaPresion += BARO.readPressure(); // Lecturas en kPa
    delay(100);
  }
  presionBase = sumaPresion / 10.0;
  Serial.print("Calibración completada. Presión base: ");
  Serial.print(presionBase);
  Serial.println(" kPa (Altitud inicial ajustada a 0m)");
  Serial.println("--------------------------------------------------\n");

  delay(1000);
}

void loop() {
  // --- LECTURA DE LA IMU ---
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az); // Retorna en Gs
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz); // Retorna en grados/segundo
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz); // Retorna en microteslas (uT)
  }

  // --- LECTURA DEL BARÓMETRO ---
  float presion = BARO.readPressure(); // kPa
  
  // Fórmula hipsométrica para calcular altitud relativa a la presión base
  float altitud = 44330.0 * (1.0 - pow((presion / presionBase), 0.190263));

  // --- LECTURA DE TEMPERATURA Y HUMEDAD ---
  float temp = HS300x.readTemperature(); // ºC
  float hum = HS300x.readHumidity();     // %

  // --- IMPRESIÓN DE RESULTADOS ---
  Serial.print("IMU: Acc[");
  Serial.print(ax, 2); Serial.print(", ");
  Serial.print(ay, 2); Serial.print(", ");
  Serial.print(az, 2); Serial.print("] Gs | ");

  Serial.print("Gyr[");
  Serial.print(gx, 1); Serial.print(", ");
  Serial.print(gy, 1); Serial.print(", ");
  Serial.print(gz, 1); Serial.print("] º/s | ");

  Serial.print("Mag[");
  Serial.print(mx, 1); Serial.print(", ");
  Serial.print(my, 1); Serial.print(", ");
  Serial.print(mz, 1); Serial.print("] uT\n");

  Serial.print("ENVI: Presión: ");
  Serial.print(presion, 3); Serial.print(" kPa | ");
  Serial.print("Altitud: ");
  Serial.print(altitud, 1); Serial.print(" m | ");
  Serial.print("Temp: ");
  Serial.print(temp, 1); Serial.print(" ºC | ");
  Serial.print("Hum: ");
  Serial.print(hum, 1); Serial.println(" %");
  Serial.println("-----------------------------------------------------------------------------");

  delay(250); // Muestreo a 4 Hz (puedes reducir el delay para mayor frecuencia)
}