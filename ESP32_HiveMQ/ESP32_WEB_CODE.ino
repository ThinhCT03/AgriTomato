#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi
const char* ssid = "Thinhct";
const char* password = "01012003";

// HiveMQ Cloud MQTT
const char* mqtt_server = "559fa1abe5694efab30428625fcf4b8c.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "thinhlecongkg2003";
const char* mqtt_pass = "Thinh@85204569";

// MQTT Topics
const char* publish_topic = "system/data";
const char* subscribe_topic = "send/cmd";

// TLS client
WiFiClientSecure espClient;
PubSubClient client(espClient);

// UART
HardwareSerial stmSerial(2);  // UART2: GPIO16 (RX), GPIO17 (TX)
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
  stmSerial.begin(115200, SERIAL_8N1, 16, 17);  // RX, TX

  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi đã kết nối");

  espClient.setInsecure(); // Tạm thời không kiểm tra chứng chỉ

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (stmSerial.available() >= 14) {
    stmSerial.readBytes(rx_buffer, 14);

    // In toàn bộ dữ liệu nhận được từ STM
    Serial.println("========== DỮ LIỆU TỪ STM ==========");
    Serial.print("Dữ liệu HEX: ");
    for (int i = 0; i < 14; i++) {
      Serial.printf("%02X ", rx_buffer[i]);
    }
    Serial.println();

    // Giải mã dữ liệu
    uint8_t id = rx_buffer[0];
    float humidity = ((rx_buffer[1] << 8) | rx_buffer[2]) / 10.0;
    float temp = ((rx_buffer[3] << 8) | rx_buffer[4]) / 10.0;
    float lux = ((rx_buffer[5] << 8) | rx_buffer[6]) * 1.0;
    uint16_t mq135 = (rx_buffer[7] << 8) | rx_buffer[8];
    float soil_temp = ((rx_buffer[9] << 8) | rx_buffer[10]) / 10.0;
    float soil_moist = ((rx_buffer[11] << 8) | rx_buffer[12]) / 10.0;

    uint8_t relay_byte = rx_buffer[13];
    uint8_t light = relay_byte & 0x01;
    uint8_t fan   = (relay_byte >> 1) & 0x01;
    uint8_t pump  = (relay_byte >> 2) & 0x01;

    // Hiển thị dữ liệu rõ ràng
    Serial.println("======== DỮ LIỆU GỬI MQTT =========");
    Serial.printf("ID Node: %d\n", id);
    Serial.printf("Độ ẩm không khí: %.2f\n", humidity);
    Serial.printf("Nhiệt độ không khí: %.2f\n", temp);
    Serial.printf("Ánh sáng (lux): %.2f\n", lux);
    Serial.printf("Khí MQ135: %d\n", mq135);
    Serial.printf("Nhiệt độ đất: %.2f\n", soil_temp);
    Serial.printf("Độ ẩm đất: %.2f\n", soil_moist);
    Serial.printf("Trạng thái đèn: %d\n", light);
    Serial.printf("Trạng thái quạt: %d\n", fan);
    Serial.printf("Trạng thái bơm: %d\n", pump);
    Serial.print("Relay Byte (BIN): ");
    for (int i = 7; i >= 0; i--) {
      Serial.print((relay_byte >> i) & 0x01);
    }
    Serial.println();
    Serial.println("====================================");

    // Gửi MQTT
    StaticJsonDocument<256> doc;
    doc["id"] = id;
    doc["humidity"] = humidity;
    doc["temp"] = temp;
    doc["lux"] = lux;
    doc["mq135"] = mq135;
    doc["soil_temp"] = soil_temp;
    doc["soil_moist"] = soil_moist;
    doc["light"] = light;
    doc["fan"] = fan;
    doc["pump"] = pump;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);

    client.publish(publish_topic, jsonBuffer);
    Serial.println("Đã gửi MQTT:");
    Serial.println(jsonBuffer);
  }
}

