// IMPORTANTE: ESTE PROGRAMA SOLO FUNCIONA EN EL ESP32-C3
#include <Arduino.h>
#include <TinyGPSPlus.h>

HardwareSerial GPS_SERIAL(0); // Usar UART0 para el GPS
TinyGPSPlus gps;  

void setup() {
  
  delay(3600); // Espera a que el monitor serie se conecte (especialmente útil para USB CDC)
Serial.begin(115200);
Serial.println("Inicio de programa");

GPS_SERIAL.begin(9600, SERIAL_8N1, 20, 21); // RX en GPIO3, TX en GPIO1 (TX no se usa)
}

void loop() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    Serial.write(c); // Muestra la trama NMEA en el monitor serie
    gps.encode(c);   // Procesa el dato con TinyGPSPlus
  }

  // Ejemplo: mostrar latitud y longitud si hay datos válidos
  if (gps.location.isUpdated()) {
    Serial.print("Latitud: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitud: ");
    Serial.println(gps.location.lng(), 6);
    
  }
}

