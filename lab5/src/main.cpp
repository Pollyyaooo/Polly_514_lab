/***********************
 * Lab 5 – 24h Power-Saving Strategy (Strategy B)
 * ESP32-C3 + HC-SR04 + Firebase (event-driven + heartbeat)
 ***********************/

#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include "esp_sleep.h"

/********* HC-SR04 *********/
#define TRIG_PIN 3
#define ECHO_PIN 4

/********* WiFi *********/
const char* ssid = "UW MPSK";
const char* password = "qmYFYGCX6mTzEuKb";

/********* Firebase *********/
#define API_KEY "AIzaSyDNj5sMFZL120f6Z36Ivk2JDV6OZdXcjN8"
#define DATABASE_URL "https://lab5-esp-polly-default-rtdb.firebaseio.com/"
#define USER_EMAIL "qinling3@uw.edu"
#define USER_PASSWORD "20030330"

/********* Config: Strategy B *********/
// wake up every WAKE_INTERVAL_SEC to sample distance
static const uint32_t WAKE_INTERVAL_SEC = 10;

// movement detection threshold (cm)
static const int MOVEMENT_THRESH_CM = 15;

// if no movement, still send a heartbeat every HEARTBEAT_SEC
static const uint32_t HEARTBEAT_SEC = 600; // 10 minutes

// number of ultrasonic samples per wake (median helps noise)
static const int SAMPLES_PER_WAKE = 3;

// how long to wait after WiFi connect (ms)
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 8000;

/********* RTC memory: persists across deep sleep *********/
RTC_DATA_ATTR int lastDistanceCm = -1;
RTC_DATA_ATTR uint32_t secondsSinceBoot = 0;
RTC_DATA_ATTR uint32_t lastUploadSec = 0;

/********* Firebase Objects *********/
WiFiClientSecure sslClient;
using AsyncClient = AsyncClientClass;
AsyncClient asyncClient(sslClient);

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);
FirebaseApp app;
RealtimeDatabase Database;

/********* Ultrasonic *********/
long readUltrasonicCM_once() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 0;
  return (long)(duration * 0.034 / 2);
}

int median3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

int readDistanceCm() {
  long d1 = readUltrasonicCM_once();
  delay(30);
  long d2 = readUltrasonicCM_once();
  delay(30);
  long d3 = readUltrasonicCM_once();

  // simple median filter
  return median3((int)d1, (int)d2, (int)d3);
}

/********* WiFi/Firebase Upload *********/
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
  }
  return (WiFi.status() == WL_CONNECTED);
}

void disconnectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool uploadToFirebase(int distanceCm, bool moved, bool heartbeatOnly) {
  if (!connectWiFi()) {
    disconnectWiFi();
    return false;
  }

  sslClient.setInsecure();

  // init firebase fresh each time (simpler & deterministic for lab)
  initializeApp(asyncClient, app, getAuth(user_auth));
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  // allow library to progress
  uint32_t t0 = millis();
  while (!app.ready() && (millis() - t0) < 5000) {
    app.loop();
    delay(10);
  }

  if (!app.ready()) {
    disconnectWiFi();
    return false;
  }

  // write a small structured payload
  String basePath = "/lab5/motion";
  Database.set<int>(asyncClient, basePath + "/distance_cm", distanceCm);
  Database.set<bool>(asyncClient, basePath + "/moved", moved);
  Database.set<bool>(asyncClient, basePath + "/heartbeat_only", heartbeatOnly);
  Database.set<int>(asyncClient, basePath + "/t_sec", (int)secondsSinceBoot);

  // let async tasks flush a bit
  uint32_t flushStart = millis();
  while (millis() - flushStart < 300) {
    app.loop();
    delay(5);
  }

  disconnectWiFi();
  return true;
}

void goToDeepSleep(uint32_t sleepSec) {
  // schedule wake-up
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // keep WiFi off unless needed
  WiFi.mode(WIFI_OFF);

  // update "time" (coarse): each cycle is WAKE_INTERVAL_SEC
  secondsSinceBoot += WAKE_INTERVAL_SEC;

  Serial.println("=== Strategy B Wake ===");
}

void loop() {
  // 1) sample ultrasonic
  int d = readDistanceCm();
  Serial.print("Distance(cm): ");
  Serial.println(d);

  // 2) movement detection
  bool moved = false;
  if (lastDistanceCm >= 0) {
    moved = (abs(d - lastDistanceCm) >= MOVEMENT_THRESH_CM);
  }
  lastDistanceCm = d;

  // 3) decide upload policy
  bool timeForHeartbeat = (secondsSinceBoot - lastUploadSec) >= HEARTBEAT_SEC;
  bool shouldUpload = moved || timeForHeartbeat;

  if (shouldUpload) {
    bool ok = uploadToFirebase(d, moved, /*heartbeatOnly*/ !moved);
    Serial.println(ok ? "Uploaded" : "Upload failed");
    if (ok) lastUploadSec = secondsSinceBoot;
  } else {
    Serial.println("No upload this cycle");
  }

  // 4) sleep until next cycle
  Serial.println("Going to deep sleep...");
  delay(50);
  goToDeepSleep(WAKE_INTERVAL_SEC);
}
