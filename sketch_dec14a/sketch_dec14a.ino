#include <WiFi.h>
#include <MQUnifiedsensor.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
#include "DHT.h"
#include "RTClib.h"
#include <ArduinoJson.h>

#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6HN4XpHhd"
#define BLYNK_TEMPLATE_NAME "Swine Husbandry"
#define BLYNK_AUTH_TOKEN "BC-rX8jnLU_tQYgSd7K96uXGUPZRcRom"

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
const long readingInterval = 1000;

bool servoState = false;
bool pirState = false;

int initialPosition = 0;
Servo myservo;

int sched1 = 0;
int sched2 = 0;
int sched3 = 0;
float tankFoodLevel = 0;

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
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // if (rtc.lostPower()) {
  //   Serial.println("RTC lost power, let's set the time!");
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // }

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
  Blynk.syncVirtual(V5);
  Blynk.syncVirtual(V6);
  Blynk.syncVirtual(V7);
  Serial.println("** Values ****");
  Serial.println("|  humidity | temperature | tankFoodLevel(cm) | motion |            date           | servo |");
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
  int unixtime = dateData["unixtime"];

  if (
    (unixtime >= sched1 && unixtime <= sched1 + 3) || (unixtime >= sched2 && unixtime <= sched2 + 3) || (unixtime >= sched3 && unixtime <= sched3 + 3)) {
    //turn on servo
    servoState = true;
  }
  float humidity = dhtData["humidity"];
  float temperature = dhtData["temperature"];
  tankFoodLevel = mapFloat(readUltrasonic(), 0, 127.0, 100.0, 0);

  if (tankFoodLevel < 25) {
    servoState = true;
  } else if (tankFoodLevel >= 90) {
    servoState = false;
  }

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
  Serial.print(pirState ? "Detected" : "Not Detected");
  Serial.print("   |   ");
  serializeJson(dateData, Serial);
  Serial.print("   |   ");
  Serial.print(servoState ? "Open" : "Closed");
  Serial.println("   |");
}

BLYNK_WRITE(V5) {
  sched1 = param[0].asInt();
}
BLYNK_WRITE(V6) {
  sched2 = param[0].asInt();
}
BLYNK_WRITE(V7) {
  sched3 = param[0].asInt();
}

void loop() {
  unsigned long currentMillis = millis();
  Blynk.run();
  readPIR();
  if (currentMillis - previousReadingMillis >= readingInterval) {
    previousReadingMillis = currentMillis;
    sendSensor();
    controlServo();
  };
}

void controlServo() {
  if (servoState) {
    if (tankFoodLevel < 90) {
      if (initialPosition <= 180) {
        for (initialPosition; initialPosition <= 180; initialPosition += 1) {
          myservo.write(initialPosition);
        }
      }
    } else {
      servoState = false;
    }


  } else if (!servoState) {
    if (initialPosition >= 0) {
      for (initialPosition; initialPosition >= 0; initialPosition -= 1) {
        myservo.write(initialPosition);
      }
    }
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
  if (cm >= 1200) cm = 0;

  return cm > 127.0 ? 127.0 : cm;
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

  uint32_t secondsSinceMidnight = now.hour() * 3600 + now.minute() * 60 + now.second();

  DynamicJsonDocument doc(200);
  doc["year"] = now.year();
  doc["month"] = now.month();
  doc["day"] = now.day();
  doc["hour"] = now.hour();
  doc["minute"] = now.minute();
  doc["second"] = now.second();
  doc["unixtime"] = secondsSinceMidnight;
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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}