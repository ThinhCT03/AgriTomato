#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Cấu hình WiFi
const char* ssid = "Thinhct";
const char* password = "01012003";

// Cấu hình HiveMQ Cloud
const char* mqtt_server = "559fa1abe5694efab30428625fcf4b8c.s1.eu.hivemq.cloud";  // hoặc dùng broker của HiveMQ Cloud nếu có tài khoản riêng
const int mqtt_port = 8883;
const char* mqtt_user = "thinhlecongkg2003";   // Bản free không cần
const char* mqtt_pass = "Thinh@85204569";

// MQTT Topics
const char* publish_topic = "system/data";
const char* subscribe_topic = "send/cmd";

WiFiClient espClient;
PubSubClient client(espClient);

// UART với STM32
HardwareSerial stmSerial(2); // UART2: GPIO16 (RX), GPIO17 (TX)
uint8_t rx_buffer[14];

void reconnect() {
  while (!client.connected()) {
    Serial.print("Đang kết nối MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Thành công!");
      client.subscribe(subscribe_topic);
    } else {
      Serial.print("Thất bại, rc=");
      Serial.print(client.state());
      Serial.println(" thử lại sau 5s");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, subscribe_topic) == 0) {
    StaticJsonDocument<64> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("Lỗi JSON điều khiển: ");
      Serial.println(error.f_str());
      return;
    }

    int light = doc["light"] | 0;
    int fan   = doc["fan"] | 0;
    int pump  = doc["pump"] | 0;

    uint8_t control_byte = (pump << 2) | (fan << 1) | light;
    stmSerial.write(control_byte);
  }
}

void setup() {
  Serial.begin(115200);
  stmSerial.begin(9600, SERIAL_8N1, 16, 17); // UART2: RX=GPIO16, TX=GPIO17

  WiFi.begin(ssid, password);
  Serial.print("Kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi đã kết nối");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (stmSerial.available() >= 14) {
    stmSerial.readBytes(rx_buffer, 14);

    StaticJsonDocument<256> doc;
    doc["id"] = rx_buffer[0];
    doc["humidity"] = (rx_buffer[1] << 8) | rx_buffer[2];
    doc["temp"] = (rx_buffer[3] << 8) | rx_buffer[4];
    doc["lux"] = (rx_buffer[5] << 8) | rx_buffer[6];
    doc["mq135"] = (rx_buffer[7] << 8) | rx_buffer[8];
    doc["soil_temp"] = (rx_buffer[9] << 8) | rx_buffer[10];
    doc["soil_moist"] = (rx_buffer[11] << 8) | rx_buffer[12];
    doc["light"] = rx_buffer[13] & 0x01;
    doc["fan"] = (rx_buffer[13] >> 1) & 0x01;
    doc["pump"] = (rx_buffer[13] >> 2) & 0x01;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);

    client.publish(publish_topic, jsonBuffer);
  }
}
