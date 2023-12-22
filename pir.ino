#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define PIR_PIN D3
#define ALERT_PIN 4
#define sensor_topic "sensor"

const char* ssid = "esp32-cam"; //--> Enter your SSID / your WiFi network name.
const char* password = "12121212"; //--> Enter your WiFi password.
const char* mqttServer = "192.168.43.111";
const int mqttPort = 1883;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  pinMode(PIR_PIN, INPUT);
  pinMode(ALERT_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }

  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("------------");
  Serial.println();
  client.setServer(mqttServer, mqttPort);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Esp8266")) {
      Serial.println("connected");
      // Subscribe
    } else {
      delay(2000);
    }
  }
}

bool PIR_Sensor_is_stable = false;

int countdown_interval_to_stabilize_PIR_Sensor = 1000;
unsigned long lastTime_countdown_Ran;
byte countdown_to_stabilize_PIR_Sensor = 30;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if(PIR_Sensor_is_stable == false) {
    if(millis() > lastTime_countdown_Ran + countdown_interval_to_stabilize_PIR_Sensor) {
      if(countdown_to_stabilize_PIR_Sensor > 0) countdown_to_stabilize_PIR_Sensor--;
      if(countdown_to_stabilize_PIR_Sensor == 0) {
        PIR_Sensor_is_stable = true;
      }
      lastTime_countdown_Ran = millis();
    }
  }
  // put your main code here, to run repeatedly:
  if(PIR_Sensor_is_stable){
    int value = digitalRead(PIR_PIN);
    if(value == 1){
      int triggerTimes = 5;
      digitalWrite(ALERT_PIN, HIGH);
      for(int i = 0; i < triggerTimes; i++){
        client.publish(sensor_topic, String(value).c_str());
        delay(1000);
      }
      digitalWrite(ALERT_PIN, LOW);
    }
  }
}
