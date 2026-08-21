// LEGACY PROTOTYPE — not used by the current three-Uno Gureumi build.
// Robot ESP32 MVP: Wi-Fi command polling, line following, RFID arrival detection,
// obstacle stop, L298N motor control, and humidify/dehumidify relay control.
//
// Required Arduino libraries: MFRC522 (Miguel Balboa). WiFi, HTTPClient and SPI
// are included with the ESP32 Arduino core.

#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <MFRC522.h>

// -------- Network: replace before upload --------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SERVER_BASE_URL = "http://192.0.2.10:8000";  // TEST-NET example; replace locally

// -------- L298N: logical inputs only. Motors need their own power supply. --------
constexpr uint8_t LEFT_EN = 25;
constexpr uint8_t LEFT_IN1 = 26;
constexpr uint8_t LEFT_IN2 = 27;
constexpr uint8_t RIGHT_EN = 14;
constexpr uint8_t RIGHT_IN1 = 32;
constexpr uint8_t RIGHT_IN2 = 33;

// Three-channel reflective line tracker. ESP32 GPIO 34/35/39 are input-only.
constexpr uint8_t LINE_LEFT = 34;
constexpr uint8_t LINE_CENTER = 35;
constexpr uint8_t LINE_RIGHT = 39;
constexpr bool LINE_BLACK_IS_LOW = true;  // change if the module gives HIGH on black tape

// HC-SR04: ECHO is 5 V, so use a resistor divider before GPIO 17.
constexpr uint8_t ULTRASONIC_TRIG = 16;
constexpr uint8_t ULTRASONIC_ECHO = 17;
constexpr int OBSTACLE_STOP_CM = 15;

// MFRC522 must be powered at 3.3 V only.
constexpr uint8_t RFID_SS = 5;
constexpr uint8_t RFID_RST = 4;
constexpr uint8_t RFID_SCK = 18;
constexpr uint8_t RFID_MISO = 19;
constexpr uint8_t RFID_MOSI = 23;
MFRC522 rfid(RFID_SS, RFID_RST);

// Relay modules are often active LOW. Change only if yours is active HIGH.
constexpr uint8_t RELAY_HUMIDIFY = 13;
constexpr uint8_t RELAY_DEHUMIDIFY = 15;
constexpr bool RELAY_ACTIVE_LOW = true;
constexpr uint8_t STATUS_LED = 2;

// Replace these with the UID values printed by the serial monitor.
const char* UID_HOME = "AA BB CC DD";
const char* UID_ZONE1 = "11 22 33 44";
const char* UID_ZONE2 = "55 66 77 88";

constexpr unsigned long POLL_INTERVAL_MS = 1000;
constexpr unsigned long U_TURN_MS = 550;  // calibrate on the actual chassis
constexpr int BASE_SPEED = 175;
constexpr int TURN_SPEED = 130;

enum RobotState { IDLE, MOVING, RUNNING_TASK, OBSTACLE_STOP };
RobotState state = IDLE;
String currentStation = "HOME";
String targetStation = "HOME";
String targetAction = "NONE";
long lastRevision = -1;
unsigned long lastPollAt = 0;

void setRelay(uint8_t pin, bool enabled) {
  digitalWrite(pin, (enabled == RELAY_ACTIVE_LOW) ? LOW : HIGH);
}

void stopTaskModules() {
  setRelay(RELAY_HUMIDIFY, false);
  setRelay(RELAY_DEHUMIDIFY, false);
}

void setWheel(uint8_t en, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  } else if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  setWheel(LEFT_EN, LEFT_IN1, LEFT_IN2, leftSpeed);
  setWheel(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, rightSpeed);
}

void stopMotors() { drive(0, 0); }

bool isBlack(uint8_t pin) {
  const bool value = digitalRead(pin) == HIGH;
  return LINE_BLACK_IS_LOW ? !value : value;
}

void followLine() {
  const bool left = isBlack(LINE_LEFT);
  const bool center = isBlack(LINE_CENTER);
  const bool right = isBlack(LINE_RIGHT);

  if (center && !left && !right) drive(BASE_SPEED, BASE_SPEED);
  else if (left) drive(TURN_SPEED / 2, TURN_SPEED);
  else if (right) drive(TURN_SPEED, TURN_SPEED / 2);
  else drive(TURN_SPEED, -TURN_SPEED);  // search for the line; calibrate for your track
}

long obstacleDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  const long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000);
  return duration == 0 ? 999 : duration / 58;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int attempt = 0; attempt < 20 && WiFi.status() != WL_CONNECTED; ++attempt) delay(250);
}

String jsonString(const String& json, const char* key) {
  const String needle = String("\"") + key + "\":";
  int start = json.indexOf(needle);
  if (start < 0) return "";
  start = json.indexOf('"', start + needle.length());
  if (start < 0) return "";
  const int end = json.indexOf('"', start + 1);
  return end < 0 ? "" : json.substring(start + 1, end);
}

long jsonLong(const String& json, const char* key) {
  const String needle = String("\"") + key + "\":";
  const int start = json.indexOf(needle);
  return start < 0 ? -1 : json.substring(start + needle.length()).toInt();
}

int stationIndex(const String& station) {
  if (station == "HOME") return 0;
  if (station == "ZONE1") return 1;
  if (station == "ZONE2") return 2;
  return -1;
}

void turnAround() {
  // This is intentionally time-based for the MVP. Tune U_TURN_MS on your own track.
  drive(TURN_SPEED, -TURN_SPEED);
  delay(U_TURN_MS);
  stopMotors();
}

void beginMove() {
  stopTaskModules();
  if (targetStation == currentStation) {
    state = (targetAction == "NONE") ? IDLE : RUNNING_TASK;
    return;
  }
  // A HOME -> ZONE1 -> ZONE2 straight route needs a 180-degree turn for reverse travel.
  if (stationIndex(targetStation) < stationIndex(currentStation)) turnAround();
  state = MOVING;
}

void startTaskAtTarget() {
  stopMotors();
  if (targetAction == "HUMIDIFY") {
    setRelay(RELAY_HUMIDIFY, true);
    setRelay(RELAY_DEHUMIDIFY, false);
    state = RUNNING_TASK;
  } else if (targetAction == "DEHUMIDIFY") {
    setRelay(RELAY_HUMIDIFY, false);
    setRelay(RELAY_DEHUMIDIFY, true);
    state = RUNNING_TASK;
  } else {
    stopTaskModules();
    state = IDLE;
  }
}

void acceptCommand(const String& command, const String& target, const String& action, long revision) {
  if (revision == lastRevision) return;
  lastRevision = revision;
  targetStation = target;
  targetAction = action;
  Serial.printf("Command #%ld: %s -> %s / %s\n", revision, command.c_str(), target.c_str(), action.c_str());

  if (command == "RETURN_HOME") {
    targetStation = "HOME";
    targetAction = "NONE";
    if (currentStation == "HOME") {
      stopTaskModules();
      stopMotors();
      state = IDLE;
    } else beginMove();
  } else if (command == "TASK") {
    if (currentStation == targetStation) startTaskAtTarget();
    else beginMove();
  }
}

void pollServerCommand() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(String(SERVER_BASE_URL) + "/api/robot/command");
  const int status = http.GET();
  if (status == HTTP_CODE_OK) {
    const String body = http.getString();
    acceptCommand(jsonString(body, "command"), jsonString(body, "target_zone"),
                  jsonString(body, "action"), jsonLong(body, "revision"));
  } else {
    Serial.printf("Command poll failed: %d\n", status);
  }
  http.end();
}

String readTagUid() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return "";
  String uid;
  for (byte i = 0; i < rfid.uid.size; ++i) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
    if (i + 1 < rfid.uid.size) uid += " ";
  }
  uid.toUpperCase();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return uid;
}

void handleRfid() {
  const String uid = readTagUid();
  if (uid.length() == 0) return;
  Serial.printf("RFID UID: %s\n", uid.c_str());
  if (uid == UID_HOME) currentStation = "HOME";
  else if (uid == UID_ZONE1) currentStation = "ZONE1";
  else if (uid == UID_ZONE2) currentStation = "ZONE2";
  else return;

  if (currentStation == targetStation) startTaskAtTarget();
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_EN, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LINE_LEFT, INPUT); pinMode(LINE_CENTER, INPUT); pinMode(LINE_RIGHT, INPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT); pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(RELAY_HUMIDIFY, OUTPUT); pinMode(RELAY_DEHUMIDIFY, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  stopMotors();
  stopTaskModules();

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();
  connectWifi();
}

void loop() {
  connectWifi();
  if (millis() - lastPollAt >= POLL_INTERVAL_MS) {
    lastPollAt = millis();
    pollServerCommand();  // monitoring stays active during move, task, and return.
  }
  handleRfid();

  if (state == MOVING) {
    if (obstacleDistanceCm() <= OBSTACLE_STOP_CM) {
      stopMotors();
      state = OBSTACLE_STOP;
    } else {
      followLine();
    }
  } else if (state == OBSTACLE_STOP && obstacleDistanceCm() > OBSTACLE_STOP_CM) {
    state = MOVING;
  }

  digitalWrite(STATUS_LED, state == IDLE ? LOW : HIGH);
}
