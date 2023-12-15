#include <WiFi.h>
#include <MQUnifiedsensor.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
#include "DHT.h"
#include "RTClib.h"
#include <ArduinoJson.h>

#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6RT8C1O01"
#define BLYNK_TEMPLATE_NAME "test"
#define BLYNK_AUTH_TOKEN "DzsO4Pk191jBqPQAp6xGj1Susx4ji9Bq"

#include <BlynkSimpleEsp32.h>

#define RelayPin (26)

#define servoPin (18)
#define pingPin (14)
#define DHTPIN (13)
#define pirPin (27)

#define DHTTYPE DHT22

char ssid[] = "scam ni";
char pass[] = "Walakokabalo0123!";

unsigned long previousReadingMillis = 0;
const long readingInterval = 500;

bool servoState = false;
bool pirState = false;

int initialPosition = 0;
Servo myservo;

DHT dht(DHTPIN, DHTTYPE);

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  delay(10);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(RelayPin, OUTPUT);
  pinMode(pirPin, INPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 1000, 2000);
  dht.begin();


  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("** Values ****");
  Serial.println("|  humidity | temperature | tankFoodLevel(cm) | motion |            date           |");
}

void sendSensor() {
  DynamicJsonDocument dhtData = readDHT();
  DynamicJsonDocument dateData = readRTC();

  String year = dateData["year"];
  String month = dateData["month"];
  String day = dateData["day"];
  String hour = dateData["hour"];
  String minute = dateData["minute"];
  String second = dateData["second"];

  float humidity = dhtData["humidity"];
  float temperature = dhtData["temperature"];
  float tankFoodLevel = readUltrasonic();

  Blynk.virtualWrite(V0, humidity);
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, tankFoodLevel);
  Blynk.virtualWrite(V3, pirState);
  Blynk.virtualWrite(V4, servoState);

  Serial.print("|   ");
  Serial.print(humidity);
  Serial.print("   |   ");
  Serial.print(temperature);
  Serial.print("   |   ");
  Serial.print(tankFoodLevel);
  Serial.print("   |   ");
  Serial.print(pirState ? "Detected": "Not Detected");
  Serial.print("   |   ");
  serializeJson(dateData, Serial);
  Serial.println("   |");
}

void loop() {
  unsigned long currentMillis = millis();
  Blynk.run();
  readPIR();
  if (currentMillis - previousReadingMillis >= readingInterval) {
    previousReadingMillis = currentMillis;
    sendSensor();
  };
}

void controlServo() {
  for (initialPosition = 0; initialPosition <= 180; initialPosition += 1) {

    myservo.write(initialPosition);
    delay(15);
  }
  for (initialPosition = 180; initialPosition >= 0; initialPosition -= 1) {
    myservo.write(initialPosition);
    delay(15);
  }
}

double readUltrasonic() {
  long duration, inches, cm;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  cm = microsecondsToCentimeters(duration);
  return cm;
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

DynamicJsonDocument readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  DynamicJsonDocument doc(200);
  doc["humidity"] = h;
  doc["temperature"] = t;
  return doc;
}

DynamicJsonDocument readRTC() {
  DateTime now = rtc.now();


  DynamicJsonDocument doc(200);
  doc["year"] = now.year();
  doc["month"] = now.month();
  doc["day"] = now.day();
  doc["hour"] = now.hour();
  doc["minute"] = now.minute();
  doc["second"] = now.second();
  return doc;
}

void readPIR() {
  float val = digitalRead(pirPin);
  if (val == HIGH) {

    if (!pirState) {
      pirState = true;
    }
  } else {
    if (pirState) {
      pirState = false;
    }
  }
}