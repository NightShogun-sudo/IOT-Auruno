#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ====== PIN ======
#define TRIG_PIN 12
#define ECHO_PIN 11

#define FAN_PIN 4
#define PUMP_PIN 5

#define RED_PIN 6
#define GREEN_PIN 7
#define BLUE_PIN 8

#define TEMP_PIN A1
#define SOIL_PIN A3
#define PH_PIN A0

#define ESP_RX_PIN 9
#define ESP_TX_PIN 10

SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// ====== TELEMETRY / COMMAND ======
StaticJsonDocument<256> jsonDoc;
char serialBuffer[192];
uint8_t serialBufferLen = 0;

unsigned long lastSensorRead = 0;
unsigned long lastTelemetrySent = 0;
unsigned long lastEspSerialActivity = 0;

const unsigned long SENSOR_INTERVAL = 1000;
const unsigned long TELEMETRY_INTERVAL = 2000;
const unsigned long SERIAL_QUIET_HOLDOFF = 250;

// Phan lon module relay 5V kich muc LOW. Neu relay cua ban kich HIGH,
// doi RELAY_ON = HIGH va RELAY_OFF = LOW.
const uint8_t RELAY_ON = LOW;
const uint8_t RELAY_OFF = HIGH;

// ====== NGUONG ======
float phLow = 5.5f;
float phHigh = 7.5f;
float tempLow = 28.0f;
float tempHigh = 32.0f;
float waterMin = 15.0f;      // Khoang cach toi mat nuoc (cm). Lon hon muc nay => muc nuoc thap
int soilThreshold = 500;     // Giu de tuong thich schema voi dashboard
bool alertEnabled = true;
bool autoMode = true;

bool pumpState = false;
bool fanState = false;
bool manualPumpState = false;
bool manualFanState = false;

// ====== pH ======
float calibrationValue = 21.34f;
int bufferArr[10];

struct SensorData {
  float distance;
  float ph;
  float temperature;
  uint16_t soilRaw;
  uint8_t soilPercent;
  bool waterLow;
  bool phAlert;
  bool tempAlert;
};

SensorData lastData = {0.0f, 0.0f, 0.0f, 0, 0, false, false, false};
bool hasLastData = false;
float lastValidDistance = 0.0f;

void readSensors(SensorData &data);
void updateOutputs(const SensorData &data);
void updateLCD(const SensorData &data);
void sendTelemetry(const SensorData &data);
void handleSerialInput();
void applyIncomingJson(JsonObjectConst message);
void setRGB(bool redOn, bool greenOn, bool blueOn);
void writeRelay(uint8_t pin, bool enabled);

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Dat muc tat truoc khi chuyen pin sang OUTPUT de tranh relay nhay luc khoi dong.
  writeRelay(FAN_PIN, false);
  writeRelay(PUMP_PIN, false);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print(F("System Start..."));
  lcd.setCursor(0, 1);
  lcd.print(F("ESP link D9/D10"));
  delay(1500);
  lcd.clear();

  setRGB(false, false, false);
}

void loop() {
  handleSerialInput();

  unsigned long now = millis();

  // Give incoming serial commands a short exclusive window so SoftwareSerial
  // is not competing with sensor reads or telemetry writes.
  if (now - lastEspSerialActivity < SERIAL_QUIET_HOLDOFF) {
    return;
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors(lastData);
    hasLastData = true;
    updateOutputs(lastData);
    updateLCD(lastData);

    Serial.print(F("Distance: "));
    Serial.print(lastData.distance, 1);
    Serial.print(F(" | Temp: "));
    Serial.print(lastData.temperature, 1);
    Serial.print(F(" | SoilRaw: "));
    Serial.print(lastData.soilRaw);
    Serial.print(F(" | Soil%: "));
    Serial.print(lastData.soilPercent);
    Serial.print(F(" | pH: "));
    Serial.println(lastData.ph, 2);
  }

  if (hasLastData && now - lastTelemetrySent >= TELEMETRY_INTERVAL) {
    lastTelemetrySent = now;
    sendTelemetry(lastData);
  }
}

void readSensors(SensorData &data) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration > 0) {
    lastValidDistance = duration * 0.034f / 2.0f;
  } else if (!hasLastData) {
    lastValidDistance = 0.0f;
  }

  data.distance = lastValidDistance;

  long tempRawTotal = 0;
  for (int i = 0; i < 10; i++) {
    tempRawTotal += analogRead(TEMP_PIN);
    delay(2);
  }

  float tempVoltage = (tempRawTotal / 10.0f) * (5.0f / 1024.0f);
  data.temperature = tempVoltage * 100.0f;

  data.soilRaw = analogRead(SOIL_PIN);
  data.soilPercent = map(data.soilRaw, 1023, 0, 0, 100);
  data.soilPercent = constrain(data.soilPercent, 0, 100);

  long avgValue = 0;
  for (int i = 0; i < 10; i++) {
    bufferArr[i] = analogRead(PH_PIN);
    delay(10);
  }

  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (bufferArr[i] > bufferArr[j]) {
        int temp = bufferArr[i];
        bufferArr[i] = bufferArr[j];
        bufferArr[j] = temp;
      }
    }
  }

  for (int i = 2; i < 8; i++) {
    avgValue += bufferArr[i];
  }

  float voltage = avgValue * 5.0f / 1024.0f / 6.0f;
  data.ph = -5.70f * voltage + calibrationValue;

  data.waterLow = data.distance > waterMin;
  data.phAlert = (data.ph < phLow || data.ph > phHigh);
  data.tempAlert = (data.temperature < tempLow || data.temperature > tempHigh);
}

void updateOutputs(const SensorData &data) {
  if (autoMode) {
    pumpState = (data.soilRaw > soilThreshold) && !data.waterLow;

    if (data.temperature > tempHigh) {
      fanState = true;
    } else if (data.temperature < tempLow) {
      fanState = false;
    }
  } else {
    pumpState = manualPumpState;
    fanState = manualFanState;
  }

  writeRelay(PUMP_PIN, pumpState);
  writeRelay(FAN_PIN, fanState);

  if (!alertEnabled) {
    setRGB(false, false, true);
    return;
  }

  if (data.phAlert) {
    setRGB(true, false, false);
    return;
  }

  if (data.tempAlert) {
    setRGB(true, false, false);
    return;
  }

  if (data.waterLow) {
    setRGB(false, false, true);
    return;
  }

  setRGB(false, true, false);
}

void updateLCD(const SensorData &data) {
  lcd.setCursor(0, 0);
  lcd.print(F("D:"));
  lcd.print((int)data.distance);
  lcd.print(F(" T:"));
  lcd.print(data.temperature, 1);
  lcd.print(F(" "));

  lcd.setCursor(0, 1);
  lcd.print(F("pH:"));
  lcd.print(data.ph, 1);
  lcd.print(F(" P"));
  lcd.print(pumpState ? F("1") : F("0"));
  lcd.print(F(" F"));
  lcd.print(fanState ? F("1") : F("0"));
  lcd.print(F("  "));
}

void sendTelemetry(const SensorData &data) {
  jsonDoc.clear();
  jsonDoc["ph"] = data.ph;
  jsonDoc["temperature"] = data.temperature;
  jsonDoc["soilMoisture"] = data.soilRaw;
  jsonDoc["waterLevel"] = data.distance;
  jsonDoc["phLow"] = phLow;
  jsonDoc["phHigh"] = phHigh;
  jsonDoc["tempLow"] = tempLow;
  jsonDoc["tempHigh"] = tempHigh;
  jsonDoc["soilThreshold"] = soilThreshold;
  jsonDoc["waterMin"] = waterMin;
  jsonDoc["pumpState"] = pumpState;
  jsonDoc["fanState"] = fanState;
  jsonDoc["alertEnabled"] = alertEnabled;
  jsonDoc["autoMode"] = autoMode;
  jsonDoc["timestamp"] = millis();

  serializeJson(jsonDoc, espSerial);
  espSerial.println();

  Serial.print(F("TX->ESP: "));
  serializeJson(jsonDoc, Serial);
  Serial.println();
}

void handleSerialInput() {
  while (espSerial.available()) {
    char c = (char)espSerial.read();
    lastEspSerialActivity = millis();

    if (c == '\n') {
      if (serialBufferLen == 0) {
        continue;
      }

      serialBuffer[serialBufferLen] = '\0';
      jsonDoc.clear();
      DeserializationError error = deserializeJson(jsonDoc, serialBuffer);
      if (error) {
        Serial.print(F("JSON error: "));
        Serial.println(error.c_str());
      } else {
        applyIncomingJson(jsonDoc.as<JsonObjectConst>());
      }

      serialBufferLen = 0;
    } else if (c != '\r') {
      if (serialBufferLen < sizeof(serialBuffer) - 1) {
        serialBuffer[serialBufferLen++] = c;
      } else {
        serialBufferLen = 0;
      }
    }
  }
}

void applyIncomingJson(JsonObjectConst message) {
  if (!message["phLow"].isNull()) {
    phLow = message["phLow"].as<float>();
  }

  if (!message["phHigh"].isNull()) {
    phHigh = message["phHigh"].as<float>();
  }

  if (!message["tempLow"].isNull()) {
    tempLow = message["tempLow"].as<float>();
  }

  if (!message["tempHigh"].isNull()) {
    tempHigh = message["tempHigh"].as<float>();
  }

  if (!message["waterMin"].isNull()) {
    waterMin = message["waterMin"].as<float>();
  }

  if (!message["soilThreshold"].isNull()) {
    soilThreshold = message["soilThreshold"].as<int>();
  }

  if (!message["alertEnabled"].isNull()) {
    alertEnabled = message["alertEnabled"].as<bool>();
  }

  if (!message["autoMode"].isNull()) {
    autoMode = message["autoMode"].as<bool>();
  }

  if (!message["pump"].isNull()) {
    manualPumpState = message["pump"].as<bool>();
  }

  if (!message["fan"].isNull()) {
    manualFanState = message["fan"].as<bool>();
  }

  Serial.print(F("RX<-ESP: "));
  serializeJson(message, Serial);
  Serial.println();

  if (hasLastData) {
    lastData.waterLow = lastData.distance > waterMin;
    lastData.phAlert = (lastData.ph < phLow || lastData.ph > phHigh);
    lastData.tempAlert = (lastData.temperature < tempLow || lastData.temperature > tempHigh);
    updateOutputs(lastData);
    updateLCD(lastData);
    sendTelemetry(lastData);
    lastTelemetrySent = millis();
  }
}

void setRGB(bool redOn, bool greenOn, bool blueOn) {
  digitalWrite(RED_PIN, redOn ? HIGH : LOW);
  digitalWrite(GREEN_PIN, greenOn ? HIGH : LOW);
  digitalWrite(BLUE_PIN, blueOn ? HIGH : LOW);
}

void writeRelay(uint8_t pin, bool enabled) {
  digitalWrite(pin, enabled ? RELAY_ON : RELAY_OFF);
}
