#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_VL53L0X.h>
#include <NewPing.h>
#include <WiFi.h>
#include <ServoEasing.hpp>
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

const String telegram_username = TELEGRAM_USERNAME;

#define PIN_SERVO      17
#define PIN_TRIG       15
#define PIN_ECHO       23
#define PIN_BUZZ       4
#define PIN_LED_V      33
#define PIN_LED_Y      27
#define PIN_LED_R      13

const int MAX_WORLD_DIST = 160;
const int LOCK_DIST      = 80;
const int DANGER_DIST    = 40;
const int TRACK_WIDTH    = 25;

Adafruit_SH1106G display(128, 64, &Wire, -1);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
NewPing sonar(PIN_TRIG, PIN_ECHO, MAX_WORLD_DIST);
ServoEasing myServo;

volatile int  sharedDist          = 999;
volatile int  sharedAngle         = 90;
volatile bool systemReady         = false;
volatile bool isTrackingMode      = false; 
volatile bool isDangerActive      = false; 
volatile int  bootPhase           = 0;
volatile int  lastKnownDangerDist = 0;

int radarMap[181];
int laserHistory[3] = {999, 999, 999};
byte laserIdx = 0;
int detectionCount = 0;

int           currentScanTarget     = 165;
unsigned long lastDetectionTime     = 0;
int           trackingCenter        = 90;
int           trackBestAngle        = 90;
int           trackBestDist         = 999;
bool          trackSweepLeft        = true;
unsigned long lastDangerTime        = 0;

void sendLightMessage(String text) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    if (client.connect("api.callmebot.com", 80)) {
      text.replace(" ", "%20");
      text.replace("⚠️", "[ALLARME]");
      text.replace("🚨", "[INTRUSO]");
      String url = "/text.php?user=" + telegram_username + "&text=" + text;
      client.print(String("GET ") + url + " HTTP/1.1\r\nHost: api.callmebot.com\r\nConnection: close\r\n\r\n");
      unsigned long timeout = millis();
      while (client.available() == 0) {
        if (millis() - timeout > 1500) { client.stop(); return; }
      }
      client.stop();
    }
  }
}

void TaskTelegram(void * pvParameters) {
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  int tentativi = 0;
  while (WiFi.status() != WL_CONNECTED && tentativi < 20) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    tentativi++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    bootPhase = 1; systemReady = true;
    sendLightMessage("SENTINEL ONLINE");
  } else { bootPhase = 2; }

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  bootPhase = 3;

  bool inDangerZoneLocal = false;
  unsigned long lastDangerTimeLocal = 0;

  for (;;) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (systemReady) {
      unsigned long now = millis();
      if (isDangerActive) {
        if (!inDangerZoneLocal || (now - lastDangerTimeLocal > 5000)) {
          String msg;
          if (inDangerZoneLocal) msg = "Ancora rilevato: " + String(lastKnownDangerDist) + "cm";
          else msg = "INTRUSO! Angolo: " + String(sharedAngle) + "deg - Dist: " + String(lastKnownDangerDist) + "cm";
          sendLightMessage(msg);
          inDangerZoneLocal = true; lastDangerTimeLocal = now;
        }
      } else if (inDangerZoneLocal) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (!isDangerActive) {
          sendLightMessage("Zona Libera"); inDangerZoneLocal = false;
        }
      }
    }
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_config = { .timeout_ms = 30000, .idle_core_mask = (1 << 1), .trigger_panic = true };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  pinMode(PIN_LED_V, OUTPUT); pinMode(PIN_LED_Y, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);

  Wire.begin(21, 22); display.begin(0x3C, true);
  if (lox.begin()) lox.setMeasurementTimingBudgetMicroSeconds(30000);

  if (myServo.attach(PIN_SERVO, 500, 2400) != -1) {
    myServo.setEasingType(EASE_LINEAR);
    myServo.setSpeed(30); myServo.startEaseTo(15);
  }

  xTaskCreatePinnedToCore(TaskTelegram, "TelegramTask", 10000, NULL, 1, NULL, 0);
}

void loop() {
  esp_task_wdt_reset();

  if (bootPhase < 3) {
    display.clearDisplay();
    display.setTextSize(1); display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 20); display.print("SYSTEM BOOT...");
    display.display();
    return;
  }

  int currentServoAngle = myServo.getCurrentAngle();
  unsigned long now = millis();

  int rawLaser = 999;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus != 4) rawLaser = m.RangeMilliMeter / 10;
  
  if (rawLaser < 5) rawLaser = 999;

  laserHistory[laserIdx] = rawLaser;
  laserIdx = (laserIdx + 1) % 3;

  int a = laserHistory[0], b = laserHistory[1], c = laserHistory[2];
  int laserVal = (a <= b) ? ((b <= c) ? b : ((a < c) ? c : a)) : ((a <= c) ? a : ((b < c) ? c : b));

  unsigned int uS = sonar.ping_median(3);
  int sonarVal = sonar.convert_cm(uS);

  int finalDist = 999;
  if (laserVal > 0 && laserVal < LOCK_DIST) finalDist = laserVal;
  else if (sonarVal > 0 && sonarVal < MAX_WORLD_DIST) finalDist = sonarVal;

  sharedDist = finalDist;
  sharedAngle = currentServoAngle;

  if (isTrackingMode && (now - lastDetectionTime > 2000)) {
    isTrackingMode = false;
    trackBestDist = 999; trackBestAngle = trackingCenter;
    myServo.setSpeed(30);
    detectionCount = 0; 
  }

  if (finalDist > 0 && finalDist < LOCK_DIST) {
    detectionCount++;
    if (detectionCount >= 3) {
      lastDetectionTime = now;
      if (!isTrackingMode) {
        isTrackingMode = true;
        trackingCenter = currentServoAngle;
        trackBestAngle = currentServoAngle;
        trackBestDist  = finalDist;
        trackSweepLeft = true;
      }
      if(detectionCount > 10) detectionCount = 10;
    }
  } else {
    detectionCount = 0; 
  }

  if (finalDist > 0 && finalDist < DANGER_DIST && detectionCount >= 3) {
    isDangerActive = true;
    lastDangerTime = now;
    lastKnownDangerDist = finalDist;
  }
  if (isDangerActive && (now - lastDangerTime > 2000)) {
    isDangerActive = false;
  }

  if (isTrackingMode) {
    int minCone = max(15, trackingCenter - TRACK_WIDTH);
    int maxCone = min(165, trackingCenter + TRACK_WIDTH);

    if (myServo.isMoving()) {
      if (finalDist > 0 && finalDist < LOCK_DIST && finalDist < trackBestDist) {
        trackBestDist = finalDist; trackBestAngle = currentServoAngle;
      }
    } else {
      if (trackBestDist < LOCK_DIST) trackingCenter = trackBestAngle;
      trackBestDist = 999; trackBestAngle = trackingCenter;
      minCone = max(15, trackingCenter - TRACK_WIDTH);
      maxCone = min(165, trackingCenter + TRACK_WIDTH);
      trackSweepLeft = !trackSweepLeft;
      int nextTarget = trackSweepLeft ? minCone : maxCone;
      myServo.setSpeed(65); myServo.startEaseTo(nextTarget);
    }
  } else {
    if (!myServo.isMoving()) {
      currentScanTarget = (currentScanTarget >= 160) ? 15 : 165;
      myServo.setSpeed(30); myServo.startEaseTo(currentScanTarget);
    }
  }

  if (finalDist > 0 && finalDist < MAX_WORLD_DIST) radarMap[currentServoAngle] = finalDist;
  else radarMap[currentServoAngle] = 0;

  digitalWrite(PIN_LED_V, LOW); digitalWrite(PIN_LED_Y, LOW); digitalWrite(PIN_LED_R, LOW); digitalWrite(PIN_BUZZ, LOW);

  if (isDangerActive) {
    digitalWrite(PIN_LED_R, HIGH); if ((millis() / 100) % 2 == 0) digitalWrite(PIN_BUZZ, HIGH);
    drawDanger(lastKnownDangerDist);
  } else if (isTrackingMode) {
    digitalWrite(PIN_LED_Y, HIGH);
    drawTracking(finalDist < LOCK_DIST ? finalDist : lastKnownDangerDist);
  } else {
    digitalWrite(PIN_LED_V, HIGH);
    drawScanning(currentServoAngle);
  }
  delay(10);
}

void drawScanning(int angle) {
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("SCANNING");
  display.drawCircle(64, 64, 60, SH110X_WHITE); display.drawCircle(64, 64, 30, SH110X_WHITE);
  for (int i = 0; i < 181; i++) {
    if (radarMap[i] > 0) {
      float rad = (180 - i) * PI / 180.0;
      int distPx = map(radarMap[i], 0, MAX_WORLD_DIST, 0, 60);
      int px = 64 + distPx * cos(rad), py = 64 - distPx * sin(rad);
      if (radarMap[i] < LOCK_DIST) display.drawCircle(px, py, 1, SH110X_WHITE);
      else display.drawPixel(px, py, SH110X_WHITE);
    }
  }
  float lineRad = (180 - angle) * PI / 180.0;
  display.drawLine(64, 64, 64 + 60 * cos(lineRad), 64 - 60 * sin(lineRad), SH110X_WHITE);
  display.display();
}

void drawTracking(int d) {
  display.clearDisplay(); display.drawRect(0, 0, 128, 64, SH110X_WHITE);
  display.drawLine(64, 10, 64, 54, SH110X_WHITE); display.drawLine(30, 32, 98, 32, SH110X_WHITE);
  display.fillCircle(64, 32, 4, SH110X_WHITE);
  display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(4, 4); display.print("TRACKING");
  display.setCursor(4, 54); display.print("DST:"); display.print(d); display.print("cm");
  if (isTrackingMode) { display.setCursor(90, 4); display.print("[LOCK]"); }
  display.display();
}

void drawDanger(int d) {
  display.clearDisplay(); display.fillRect(0, 0, 128, 64, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setTextSize(2); display.setCursor(25, 15); display.print("DANGER");
  display.setTextSize(2); display.setCursor(35, 40); display.print(d); display.print(" cm");
  display.setTextColor(SH110X_WHITE);
  display.display();
}