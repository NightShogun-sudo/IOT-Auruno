#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// ================= WIFI =================
const char* WIFI_SSID     = "ss A34 ";
const char* WIFI_PASSWORD = "12345678";

// ================= MQTT =================
const char* MQTT_SERVER   = "10.176.228.181";
const uint16_t MQTT_PORT  = 1883;

// ================= TOPIC =================
const char* TOPIC_TELEMETRY = "iot/khoa/telemetry";
const char* TOPIC_CONTROL   = "iot/khoa/control";
const char* TOPIC_SETTINGS  = "iot/khoa/settings";

// ================= SERIAL =================
// RX = D2 (GPIO4), TX = D1 (GPIO5)
SoftwareSerial arduinoSerial(4, 5);

// ================= CLIENT =================
WiFiClient espClient;
PubSubClient client(espClient);

// ================= BUFFER =================
String serialBuffer = "";
unsigned long lastReconnectAttempt = 0;
unsigned long lastArduinoByteAt = 0;

const unsigned long SERIAL_QUIET_BEFORE_TX_MS = 250;
const unsigned long COMMAND_ACK_TIMEOUT_MS = 1200;
const uint8_t MAX_COMMAND_RETRIES = 3;
const uint8_t MAX_PENDING_SERIAL_MESSAGES = 12;

struct PendingSerialMessage {
  String payload;
  bool awaitingAck = false;
  uint8_t attempts = 0;
  unsigned long sentAt = 0;
};

PendingSerialMessage pendingMessages[MAX_PENDING_SERIAL_MESSAGES];
uint8_t pendingHead = 0;
uint8_t pendingTail = 0;
uint8_t pendingCount = 0;

bool enqueueSerialMessage(const String& message);
void enqueueSingleKeyMessage(JsonObjectConst source, const char* key);
void queueIncomingMqttMessage(const char* topic, const String& message);
void processPendingSerialMessages();
void clearCurrentPendingMessage(const char* reason);

// ================= WIFI =================
void setup_wifi() {
  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 15000) {
      Serial.println("\nWiFi timeout!");
      return;
    }
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ================= MQTT CALLBACK =================
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("[MQTT] ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(message);

  // Queue short serial commands instead of writing immediately while
  // telemetry may already be flowing in the opposite direction.
  queueIncomingMqttMessage(topic, message);
}

// ================= MQTT CONNECT =================
boolean reconnect() {
  if (client.connected()) return true;

  Serial.print("MQTT connecting...");

  String clientId = "ESP8266-";
  clientId += String(random(0xffff), HEX);

  if (client.connect(clientId.c_str())) {
    Serial.println("OK");

    client.subscribe(TOPIC_CONTROL);
    client.subscribe(TOPIC_SETTINGS);

    return true;
  } else {
    Serial.print("fail rc=");
    Serial.println(client.state());
    return false;
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(9600);

  setup_wifi();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(512);
}

// ================= LOOP =================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      reconnect();
    }
  } else {
    client.loop();
  }

  while (arduinoSerial.available()) {
    char c = arduinoSerial.read();
    lastArduinoByteAt = millis();

    if (c == '\n') {
      if (serialBuffer.length() > 0) {
        StaticJsonDocument<256> telemetryDoc;
        bool telemetryOk = deserializeJson(telemetryDoc, serialBuffer) == DeserializationError::Ok;
        boolean ok = client.publish(TOPIC_TELEMETRY, serialBuffer.c_str());

        Serial.print("Send: ");
        Serial.print(serialBuffer);
        Serial.print(" -> ");
        Serial.println(ok ? "OK" : "FAIL");

        if (telemetryOk &&
            pendingCount > 0 &&
            pendingMessages[pendingHead].awaitingAck &&
            millis() - pendingMessages[pendingHead].sentAt <= COMMAND_ACK_TIMEOUT_MS) {
          clearCurrentPendingMessage("ack");
        }
      }

      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;

      if (serialBuffer.length() > 256) {
        serialBuffer = "";
      }
    }
  }

  processPendingSerialMessages();
}

bool enqueueSerialMessage(const String& message) {
  if (pendingCount >= MAX_PENDING_SERIAL_MESSAGES) {
    Serial.print("[ARDUINO CMD] queue full, drop: ");
    Serial.println(message);
    return false;
  }

  pendingMessages[pendingTail].payload = message;
  pendingMessages[pendingTail].awaitingAck = false;
  pendingMessages[pendingTail].attempts = 0;
  pendingMessages[pendingTail].sentAt = 0;
  pendingTail = (pendingTail + 1) % MAX_PENDING_SERIAL_MESSAGES;
  pendingCount++;
  return true;
}

void enqueueSingleKeyMessage(JsonObjectConst source, const char* key) {
  if (source[key].isNull()) {
    return;
  }

  StaticJsonDocument<96> messageDoc;
  messageDoc[key] = source[key];

  String message;
  serializeJson(messageDoc, message);
  enqueueSerialMessage(message);
}

void queueIncomingMqttMessage(const char* topic, const String& message) {
  StaticJsonDocument<256> incomingDoc;
  DeserializationError error = deserializeJson(incomingDoc, message);

  if (error) {
    enqueueSerialMessage(message);
    return;
  }

  JsonObjectConst root = incomingDoc.as<JsonObjectConst>();

  if (strcmp(topic, TOPIC_SETTINGS) == 0) {
    enqueueSingleKeyMessage(root, "phLow");
    enqueueSingleKeyMessage(root, "phHigh");
    enqueueSingleKeyMessage(root, "tempLow");
    enqueueSingleKeyMessage(root, "tempHigh");
    enqueueSingleKeyMessage(root, "soilThreshold");
    enqueueSingleKeyMessage(root, "waterMin");
    enqueueSingleKeyMessage(root, "alertEnabled");
    enqueueSingleKeyMessage(root, "autoMode");
    return;
  }

  enqueueSingleKeyMessage(root, "autoMode");
  enqueueSingleKeyMessage(root, "alertEnabled");
  enqueueSingleKeyMessage(root, "pump");
  enqueueSingleKeyMessage(root, "fan");
}

void processPendingSerialMessages() {
  if (pendingCount == 0) {
    return;
  }

  PendingSerialMessage &current = pendingMessages[pendingHead];
  unsigned long now = millis();

  if (current.awaitingAck) {
    if (now - current.sentAt > COMMAND_ACK_TIMEOUT_MS) {
      current.awaitingAck = false;

      if (current.attempts >= MAX_COMMAND_RETRIES) {
        clearCurrentPendingMessage("timeout");
      }
    }

    return;
  }

  if (now - lastArduinoByteAt < SERIAL_QUIET_BEFORE_TX_MS) {
    return;
  }

  current.attempts++;
  current.awaitingAck = true;
  current.sentAt = now;

  Serial.print("[ARDUINO CMD] ");
  Serial.println(current.payload);
  arduinoSerial.println(current.payload);
}

void clearCurrentPendingMessage(const char* reason) {
  if (pendingCount == 0) {
    return;
  }

  Serial.print("[ARDUINO CMD] done ");
  Serial.print(reason);
  Serial.print(": ");
  Serial.println(pendingMessages[pendingHead].payload);

  pendingMessages[pendingHead].payload = "";
  pendingMessages[pendingHead].awaitingAck = false;
  pendingMessages[pendingHead].attempts = 0;
  pendingMessages[pendingHead].sentAt = 0;
  pendingHead = (pendingHead + 1) % MAX_PENDING_SERIAL_MESSAGES;
  pendingCount--;
}
