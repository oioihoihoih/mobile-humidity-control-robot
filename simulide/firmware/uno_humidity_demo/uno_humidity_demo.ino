// SimulIDE demonstration firmware for the mobile humidity robot.
// DHT22 data pin: D2
// The eight LEDs stand in for the two L298N channels and the two task relays.

#include <DHT.h>

constexpr byte DHT_PIN = 2;
constexpr byte DHT_TYPE = DHT22;
DHT dht(DHT_PIN, DHT_TYPE);

constexpr byte LEFT_EN = 3;
constexpr byte LEFT_IN1 = 4;
constexpr byte LEFT_IN2 = 5;
constexpr byte RIGHT_EN = 6;
constexpr byte RIGHT_IN1 = 7;
constexpr byte RIGHT_IN2 = 8;
constexpr byte HUMIDIFY_OUTPUT = 9;
constexpr byte DEHUMIDIFY_OUTPUT = 10;

constexpr float LOW_HUMIDITY = 40.0;
constexpr float HIGH_HUMIDITY = 60.0;

void stopRobot() {
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void driveForwardSignal() {
  // This is the exact IN/EN pattern later sent to the L298N in real hardware.
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_EN, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
}

void setup() {
  Serial.begin(9600);
  dht.begin();
  for (byte pin = LEFT_EN; pin <= DEHUMIDIFY_OUTPUT; ++pin) pinMode(pin, OUTPUT);
  stopRobot();
  digitalWrite(HUMIDIFY_OUTPUT, LOW);
  digitalWrite(DEHUMIDIFY_OUTPUT, LOW);
}

void loop() {
  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT22 read failed");
    stopRobot();
    delay(2500);
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.print(" C, Humidity: ");
  Serial.print(humidity, 1);
  Serial.print(" % -> ");

  if (humidity < LOW_HUMIDITY) {
    driveForwardSignal();
    digitalWrite(HUMIDIFY_OUTPUT, HIGH);
    digitalWrite(DEHUMIDIFY_OUTPUT, LOW);
    Serial.println("HUMIDIFY / move signal ON");
  } else if (humidity > HIGH_HUMIDITY) {
    driveForwardSignal();
    digitalWrite(HUMIDIFY_OUTPUT, LOW);
    digitalWrite(DEHUMIDIFY_OUTPUT, HIGH);
    Serial.println("DEHUMIDIFY / move signal ON");
  } else {
    stopRobot();
    digitalWrite(HUMIDIFY_OUTPUT, LOW);
    digitalWrite(DEHUMIDIFY_OUTPUT, LOW);
    Serial.println("NORMAL / robot idle");
  }

  delay(2500);  // DHT22 should not be polled more frequently than every 2 seconds.
}
