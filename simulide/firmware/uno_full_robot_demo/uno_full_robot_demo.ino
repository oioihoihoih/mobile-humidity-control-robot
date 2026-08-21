#include <DHT.h>

// SimulIDE full-system demonstration firmware
// Target board: Arduino Uno
// The two DHT22 components represent the sensor zones and their ESP32/server
// telemetry path. Station and line sensors are manual push-button proxies.

const byte DHT_TYPE = DHT22;
const byte DHT1_PIN = 2;
// L298N-equivalent dual H-bridge inputs. These pins drive MOSFET gates only;
// motor current is supplied by the separate 5 V motor rail in the circuit.
const byte LEFT_IN1 = 3;
const byte LEFT_IN2 = 4;
const byte RIGHT_IN1 = 5;
const byte RIGHT_IN2 = 6;
const byte ULTRASONIC_TRIG = 7;
const byte ULTRASONIC_ECHO = 8;
const byte DHT2_PIN = 9;
const byte LINE_LEFT_PIN = 10;
const byte LINE_CENTER_PIN = 11;
const byte LINE_RIGHT_PIN = 12;
const byte BUZZER_PIN = 13;
const byte HUMIDIFY_RELAY_PIN = A3;
const byte DEHUMIDIFY_RELAY_PIN = A4;
const byte STATION_HOME_PIN = A0;
const byte STATION_ZONE1_PIN = A1;
const byte STATION_ZONE2_PIN = A2;

const float HUMIDITY_LOW = 40.0;
const float HUMIDITY_HIGH = 60.0;
const unsigned long SENSOR_INTERVAL_MS = 2500;
const unsigned long OBSTACLE_INTERVAL_MS = 350;
// Simulation-only travel proxy. The physical robot uses the RFID station
// inputs above; SimulIDE automatically marks arrival after this interval so
// the complete MOVE -> TASK sequence can be demonstrated without a latching
// RFID component.
const unsigned long SIMULATED_TRAVEL_MS = 6000;

DHT zone1Dht(DHT1_PIN, DHT_TYPE);
DHT zone2Dht(DHT2_PIN, DHT_TYPE);

enum Station : byte { HOME, ZONE1, ZONE2 };
enum RobotMode : byte { IDLE, MOVE_TO_TARGET, HUMIDIFY, DEHUMIDIFY, RETURNING, BLOCKED };

Station currentStation = HOME;
Station targetStation = HOME;
RobotMode robotMode = IDLE;
float zoneHumidity[2] = {NAN, NAN};
unsigned long violationStartedAt[2] = {0, 0};
unsigned long lastSensorAt = 0;
unsigned long lastObstacleAt = 0;
unsigned long travelStartedAt = 0;

const char* stationName(Station station) {
  if (station == ZONE1) return "ZONE1";
  if (station == ZONE2) return "ZONE2";
  return "HOME";
}

const char* modeName(RobotMode mode) {
  switch (mode) {
    case MOVE_TO_TARGET: return "MOVE";
    case HUMIDIFY: return "HUMIDIFY";
    case DEHUMIDIFY: return "DEHUMIDIFY";
    case RETURNING: return "RETURN";
    case BLOCKED: return "BLOCKED";
    default: return "IDLE";
  }
}

void motorsStop() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void driveForward() {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
}

void turnLeft() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
}

void turnRight() {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void taskOutputsOff() {
  digitalWrite(HUMIDIFY_RELAY_PIN, LOW);
  digitalWrite(DEHUMIDIFY_RELAY_PIN, LOW);
}

bool zoneNeedsAction(byte zone) {
  return !isnan(zoneHumidity[zone]) &&
         (zoneHumidity[zone] < HUMIDITY_LOW || zoneHumidity[zone] > HUMIDITY_HIGH);
}

float zoneScore(byte zone, unsigned long now) {
  if (!zoneNeedsAction(zone)) return 0.0;

  float deviation = zoneHumidity[zone] < HUMIDITY_LOW
      ? HUMIDITY_LOW - zoneHumidity[zone]
      : zoneHumidity[zone] - HUMIDITY_HIGH;
  // Severity is primary; every 10 seconds of persistence adds 1 point.
  float persistence = violationStartedAt[zone] == 0
      ? 0.0
      : (now - violationStartedAt[zone]) / 10000.0;
  return deviation + persistence;
}

void chooseTarget(unsigned long now) {
  float score1 = zoneScore(0, now);
  float score2 = zoneScore(1, now);

  if (score1 == 0.0 && score2 == 0.0) {
    targetStation = HOME;
    return;
  }
  targetStation = score1 >= score2 ? ZONE1 : ZONE2;
}

void readStationButtons() {
  // Buttons connect an INPUT_PULLUP pin to GND, emulating RFID arrival tags.
  if (digitalRead(STATION_HOME_PIN) == LOW) currentStation = HOME;
  if (digitalRead(STATION_ZONE1_PIN) == LOW) currentStation = ZONE1;
  if (digitalRead(STATION_ZONE2_PIN) == LOW) currentStation = ZONE2;
}

void readStationSerial() {
  // SimulIDE's push buttons are momentary and can be shorter than a visible
  // frame. The same arrival events can therefore be injected from the serial
  // monitor: h = HOME, 1 = ZONE1, 2 = ZONE2.
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'h' || command == 'H') currentStation = HOME;
    if (command == '1') currentStation = ZONE1;
    if (command == '2') currentStation = ZONE2;
  }
}

void readHumidityAndSchedule(unsigned long now) {
  if (now - lastSensorAt < SENSOR_INTERVAL_MS) return;
  lastSensorAt = now;

  float h1 = zone1Dht.readHumidity();
  float h2 = zone2Dht.readHumidity();
  if (!isnan(h1)) zoneHumidity[0] = h1;
  if (!isnan(h2)) zoneHumidity[1] = h2;

  for (byte zone = 0; zone < 2; zone++) {
    if (zoneNeedsAction(zone)) {
      if (violationStartedAt[zone] == 0) violationStartedAt[zone] = now;
    } else {
      violationStartedAt[zone] = 0;
    }
  }
  chooseTarget(now);

  Serial.print("H1=");
  Serial.print(zoneHumidity[0], 1);
  Serial.print(" H2=");
  Serial.print(zoneHumidity[1], 1);
  Serial.print(" target=");
  Serial.print(stationName(targetStation));
  Serial.print(" current=");
  Serial.println(stationName(currentStation));
}

long obstacleDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000);
  if (duration == 0) return 999;
  return duration / 58;
}

bool obstacleDetected(unsigned long now) {
  if (now - lastObstacleAt < OBSTACLE_INTERVAL_MS) return false;
  lastObstacleAt = now;
  return obstacleDistanceCm() < 15;
}

void lineFollow() {
  // Push buttons are active-low line sensor proxies. No button means centred.
  bool leftOnLine = digitalRead(LINE_LEFT_PIN) == LOW;
  bool centerOnLine = digitalRead(LINE_CENTER_PIN) == LOW;
  bool rightOnLine = digitalRead(LINE_RIGHT_PIN) == LOW;

  if (leftOnLine && !rightOnLine) turnLeft();
  else if (rightOnLine && !leftOnLine) turnRight();
  else if (centerOnLine || (!leftOnLine && !rightOnLine)) driveForward();
  else driveForward();
}

void applyStateMachine(unsigned long now) {
  taskOutputsOff();

  if (targetStation == HOME) {
    if (currentStation == HOME) {
      robotMode = IDLE;
      motorsStop();
      travelStartedAt = 0;
    } else {
      if (travelStartedAt == 0) travelStartedAt = now;
      if (now - travelStartedAt >= SIMULATED_TRAVEL_MS) {
        currentStation = HOME;
        travelStartedAt = 0;
        robotMode = IDLE;
        motorsStop();
      } else {
        robotMode = RETURNING;
        lineFollow();
      }
    }
    return;
  }

  if (currentStation != targetStation) {
    if (travelStartedAt == 0) travelStartedAt = now;
    if (obstacleDetected(now)) {
      robotMode = BLOCKED;
      motorsStop();
      tone(BUZZER_PIN, 1800, 120);
    } else if (now - travelStartedAt >= SIMULATED_TRAVEL_MS) {
      currentStation = targetStation;
      travelStartedAt = 0;
      motorsStop();
    } else {
      robotMode = MOVE_TO_TARGET;
      lineFollow();
    }
    return;
  }

  travelStartedAt = 0;
  motorsStop();
  byte zone = targetStation == ZONE1 ? 0 : 1;
  if (zoneHumidity[zone] < HUMIDITY_LOW) {
    robotMode = HUMIDIFY;
    digitalWrite(HUMIDIFY_RELAY_PIN, HIGH);
  } else {
    robotMode = DEHUMIDIFY;
    digitalWrite(DEHUMIDIFY_RELAY_PIN, HIGH);
  }
}

void setup() {
  Serial.begin(9600);
  zone1Dht.begin();
  zone2Dht.begin();

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(HUMIDIFY_RELAY_PIN, OUTPUT);
  pinMode(DEHUMIDIFY_RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LINE_LEFT_PIN, INPUT_PULLUP);
  pinMode(LINE_CENTER_PIN, INPUT_PULLUP);
  pinMode(LINE_RIGHT_PIN, INPUT_PULLUP);
  pinMode(STATION_HOME_PIN, INPUT_PULLUP);
  pinMode(STATION_ZONE1_PIN, INPUT_PULLUP);
  pinMode(STATION_ZONE2_PIN, INPUT_PULLUP);

  motorsStop();
  taskOutputsOff();
  Serial.println("Mobile humidity robot - 4 wheel simulation ready");
}

void loop() {
  unsigned long now = millis();
  readStationButtons();
  readStationSerial();
  readHumidityAndSchedule(now);
  applyStateMachine(now);

  static unsigned long lastStatusAt = 0;
  if (now - lastStatusAt > 1000) {
    lastStatusAt = now;
    Serial.print("MODE=");
    Serial.print(modeName(robotMode));
    Serial.print(" target=");
    Serial.println(stationName(targetStation));
  }
}
