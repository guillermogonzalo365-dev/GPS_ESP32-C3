// IMPORTANTE: ESTE PROGRAMA SOLO FUNCIONA EN EL ESP32-C3
#include <Arduino.h>

#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <SoftwareSerial.h> //si necesitamos más uart
#include <PubSubClient.h>

const char *ssid = "PruebaESP32";
const char *password = "penepene";
const char *mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;
const char *accessToken = "ZCgOd23zZqLhjU0LnSUa";

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    delay(500);
}

void reconnect()
{
  while (!client.connected())
  {
    if (client.connect("ESP32", accessToken, NULL))
      break;
    delay(2000);
  }
}



HardwareSerial GPS_SERIAL(0); // Usar UART0 para el GPS
TinyGPSPlus gps;

void setup()
{
#ifdef CONFIG_BT_ENABLED
  btStop();
#endif

digitalWrite(8, HIGH); // Apaga el LED inicialmente
pinMode(8, OUTPUT);


  delay(3600); // Espera a que el monitor serie se conecte (especialmente útil para USB CDC)
  Serial.begin(115200);
  Serial.println("Inicio de programa");

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port); // Configura el servidor MQTT

  GPS_SERIAL.begin(9600, SERIAL_8N1, 20, 21); // RX en GPIO20, TX en GPIO21 (TX no se usa)
}

void loop()
{
  while (GPS_SERIAL.available() > 0)
  {
    char c = GPS_SERIAL.read();
    // Serial.write(c); // Muestra la trama NMEA en el monitor serie (comentar si no se desea)
    gps.encode(c); // Procesa el dato con TinyGPSPlus
  }

  if(WiFi.status() == WL_CONNECTED){
    digitalWrite(8, LOW); // Enciende el LED si la conexión es exitosa
  }else{
    digitalWrite(8, HIGH); // Apaga el LED si no está conectado
  }

  // Ejemplo: mostrar latitud y longitud si hay datos válidos
  if (gps.location.isValid())
  {

    Serial.print("Latitud: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitud: ");
    Serial.println(gps.location.lng(), 6);
    delay(100);
  

    if (!client.connected()) reconnect();
  client.loop();

  // --- Aquí obtén coordenadas reales desde el GPS ---
  float lat = gps.location.lat();
  float lon = gps.location.lng();
  // ---------------------------------------------------

  String payload = "{\"latitude\":";
  payload += String(lat, 6);
  payload += ",\"longitude\":";
  payload += String(lon, 6);
  payload += "}";

  client.publish("v1/devices/me/telemetry", payload.c_str());
  delay(5000);
  }
}