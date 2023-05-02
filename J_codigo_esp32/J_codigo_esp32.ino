#include <Separador.h>
Separador s;
#define RXp2 16
#define TXp2 17
#include <ArduinoJson.h>
#include <WiFi.h>

#include <AsyncMqttClient.h>
#include <Wire.h>


//replace with your network credentials
#define WIFI_SSID "JUAN"
#define WIFI_PASSWORD "10203040"

// Raspberry Pi Mosquitto MQTT Broker
//#define MQTT_HOST IPAddress(192, 168, 18, 181)
#define MQTT_HOST IPAddress(192, 168, 1, 14)// ip raspberry pi
#define MQTT_PORT 1883

//MQTT Topics
//#define MQTT_PUB_ID "esp32/data/id"
//#define MQTT_PUB_FECHA  "esp32/data/fecha"
#define MQTT_PUB_VOLT "esp32/data/voltaje"
#define MQTT_PUB_AMPE  "esp32/data/amperaje"
#define MQTT_PUB_COLL  "esp32/data/cadena"

///////////////////////////////////////////////


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;
const long interval = 10000;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %dn", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  //Serial.print("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void setup() {
   Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  delay(1000);

  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

void loop() {
// String cadena = fecha + "," + tot_vol + "," + tot_corr;
   // Serial.print("Cadena: "); Serial.println(cadena);
   
    if (Serial2.available())
  {
    String datos_recibidos = "";
    String fecha = "";
    String voltaje = "";
    String corriente = "";
    datos_recibidos = Serial2.readStringUntil('\n');
    Serial.println(datos_recibidos);
    int tam = datos_recibidos.length();
    Serial.println(tam);
    if (tam > 25) {
      fecha = s.separa(datos_recibidos, ',', 0);
      voltaje = s.separa(datos_recibidos, ',', 1);
      corriente = s.separa(datos_recibidos, ',', 2);
      Serial.println("Fecha: "+fecha);
      Serial.println("Voltaje "+ voltaje);
      Serial.println("Corriente "+corriente);
      enviar_datos_node(voltaje,corriente );
      delay(100);
       enviar_mongobd(datos_recibidos);
    } else {
      //Serial.println("Es menor");
    }
  }
    
 
 
}


void enviar_datos_node(String value_voltaje, String value_corriente ) {
  // Publish an MQTT message on topic esp32/bme280/temperature
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_VOLT, 1, true, value_voltaje.c_str());
  // Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_VOLT, packetIdPub3);
  //Serial.printf("Message:", value_voltaje);

  // Publish an MQTT message on topic esp32/bme280/humidity
  uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_AMPE, 1, true, value_corriente.c_str());
  //Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_AMPE, packetIdPub4);
  //Serial.printf("Message:", value_amperaje);

}
void enviar_mongobd(String datos) {
  Serial.print("Datos a enviar cadena: ");Serial.println(datos);
  uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_COLL, 1, true, datos.c_str());
  //Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_COLL, packetIdPub5);
  //Serial.printf("Message:", value_amperaje);
}
