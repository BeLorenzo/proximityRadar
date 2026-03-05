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

// --- CONFIGURAZIONE PIN ---
#define PIN_SERVO      17
#define PIN_TRIG       15
#define PIN_ECHO       23
#define PIN_BUZZ       4
#define PIN_LED_V      33
#define PIN_LED_Y      27
#define PIN_LED_R      13

// --- COSTANTI LOGICA E SOGLIE ---
const String telegram_username      = TELEGRAM_USERNAME;
const int INVALID_DIST              = 999;
const int MIN_VALID_DIST            = 5;
const int MAX_WORLD_DIST            = 250;
const int LOCK_DIST                 = 80;
const int DANGER_DIST               = 40;
const int TRACK_WIDTH               = 25;
const int SERVO_MIN_ANGLE           = 15;
const int SERVO_MAX_ANGLE           = 165;
const int SERVO_CENTER_ANGLE        = 90;
const int SERVO_SPEED_SCAN          = 30;
const int SERVO_SPEED_TRACK         = 65;
const int RADAR_MAP_SIZE            = 181;
const int LASER_HISTORY_SIZE        = 3;
const unsigned long TRACKING_TIMEOUT_MS = 3000;
const unsigned long DANGER_TIMEOUT_MS   = 1000;
const unsigned long DISPLAY_UPDATE_MS   = 50;
const int TRACK_MIN_DETECTIONS      = 6;
const int TRACK_MAX_DETECTIONS      = 10;
const int DANGER_MIN_DETECTIONS     = 3;

// --- COSTANTI DISPLAY ---
const int OLED_WIDTH                = 128;
const int OLED_HEIGHT               = 64;
const int SCAN_CENTER_X             = 64;
const int SCAN_CENTER_Y             = 64;
const int SCAN_RADIUS_OUTER         = 60;
const int SCAN_RADIUS_INNER         = 30;
const int TRACK_CROSSHAIR_X         = 64;
const int TRACK_CROSSHAIR_Y         = 32;
const int TRACK_CROSSHAIR_R         = 4;

// --- OGGETTI HARDWARE ---
Adafruit_SH1106G display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
NewPing sonar(PIN_TRIG, PIN_ECHO, MAX_WORLD_DIST);
ServoEasing myServo;

// --- VARIABILI GLOBALI E CONDIVISE ---
volatile int  sharedDist            = INVALID_DIST;
volatile int  sharedAngle           = SERVO_CENTER_ANGLE;
volatile bool systemReady           = false;
volatile bool isTrackingMode        = false; 
volatile bool isDangerActive        = false; 
volatile int  bootPhase             = 0;
volatile int  lastKnownDangerDist   = 0;

int radarMap[RADAR_MAP_SIZE]; 
int laserHistory[LASER_HISTORY_SIZE] = {INVALID_DIST, INVALID_DIST, INVALID_DIST}; 
byte laserIdx = 0;
int detectionCount = 0;

int           currentScanTarget     = SERVO_MAX_ANGLE;
unsigned long lastDetectionTime     = 0;
int           trackingCenter        = SERVO_CENTER_ANGLE;
int           trackBestAngle        = SERVO_CENTER_ANGLE;
int           trackBestDist         = INVALID_DIST;
bool          trackSweepLeft        = true;
unsigned long lastDangerTime        = 0;
unsigned long lastDisplayUpdate     = 0;

// --- FUNZIONI DI SUPPORTO ---
int getMedian(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void sendLightMessage(String text) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    if (client.connect("api.callmebot.com", 80)) {
      text.replace(" ", "%20");
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

// --- TASK TELEGRAM (Core 0) ---
void TaskTelegram(void * pvParameters) {
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  int tentativi = 0;
  while (WiFi.status() != WL_CONNECTED && tentativi < 20) {
    vTaskDelay(500);
    tentativi++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    bootPhase = 1; systemReady = true;
    sendLightMessage("SENTINEL ONLINE");
  } else { bootPhase = 2; }

  vTaskDelay(2000);
  bootPhase = 3;

  bool inDangerZoneLocal = false;
  unsigned long lastDangerTimeLocal = 0;

  for (;;) {
    vTaskDelay(100);
    if (systemReady) {
      unsigned long now = millis();
      if (isDangerActive) {
        if (!inDangerZoneLocal || (now - lastDangerTimeLocal > 5000)) {
          String msg = inDangerZoneLocal ? "Ancora rilevato: " + String(lastKnownDangerDist) + "cm" : "INTRUSO! Rilevato a " + String(lastKnownDangerDist) + "cm";
          sendLightMessage(msg);
          inDangerZoneLocal = true; lastDangerTimeLocal = now;
        }
      } else if (inDangerZoneLocal) {
        vTaskDelay(3000);
        if (!isDangerActive) {
          sendLightMessage("Zona Libera"); inDangerZoneLocal = false;
        }
      }
    }
  }
}

// --- LOGICA MODULARE (Core 1) ---
int readSensors() {
  int rawLaser = INVALID_DIST;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus != 4) rawLaser = m.RangeMilliMeter / 10;
  if (rawLaser < MIN_VALID_DIST) rawLaser = INVALID_DIST;

  laserHistory[laserIdx] = rawLaser;
  laserIdx = (laserIdx + 1) % LASER_HISTORY_SIZE;
  int laserVal = getMedian(laserHistory[0], laserHistory[1], laserHistory[2]);

  unsigned int uS = sonar.ping_median(3);
  int sonarVal = sonar.convert_cm(uS);
  if (sonarVal > 0 && sonarVal < MIN_VALID_DIST) sonarVal = INVALID_DIST; 

  if (laserVal > 0 && laserVal < LOCK_DIST) return laserVal;
  if (sonarVal > 0 && sonarVal < MAX_WORLD_DIST) return sonarVal;
  return INVALID_DIST;
}

void updateSystemState(int finalDist, unsigned long now) {
  if (isTrackingMode && (now - lastDetectionTime > TRACKING_TIMEOUT_MS)) {
    isTrackingMode = false;
    trackBestDist = INVALID_DIST; trackBestAngle = trackingCenter;
    myServo.setSpeed(SERVO_SPEED_SCAN);
    detectionCount = 0; 
  }

  if (finalDist > 0 && finalDist < LOCK_DIST) {
    detectionCount++;
    if (isTrackingMode) lastDetectionTime = now;
    
    if (detectionCount >= TRACK_MIN_DETECTIONS) {
      lastDetectionTime = now;
      if (!isTrackingMode) {
        isTrackingMode = true;
        trackingCenter = myServo.getCurrentAngle();
        trackBestAngle = trackingCenter;
        trackBestDist  = finalDist;
        trackSweepLeft = true;
      }
      if(detectionCount > TRACK_MAX_DETECTIONS) detectionCount = TRACK_MAX_DETECTIONS;
    }
  } else {
    detectionCount = 0; 
  }

  if (finalDist > 0 && finalDist < DANGER_DIST && detectionCount >= DANGER_MIN_DETECTIONS) {
    isDangerActive = true;
    lastDangerTime = now;
    lastKnownDangerDist = finalDist;
  }
  
  if (isDangerActive && (now - lastDangerTime > DANGER_TIMEOUT_MS)) {
    isDangerActive = false;
  }
}

void updateServo(int finalDist, int currentServoAngle) {
  if (isTrackingMode) {
    int minCone = max(SERVO_MIN_ANGLE, trackingCenter - TRACK_WIDTH);
    int maxCone = min(SERVO_MAX_ANGLE, trackingCenter + TRACK_WIDTH);

    if (myServo.isMoving()) {
      if (finalDist > 0 && finalDist < LOCK_DIST && finalDist < trackBestDist) {
        trackBestDist = finalDist; trackBestAngle = currentServoAngle;
      }
    } else {
      if (trackBestDist < LOCK_DIST) trackingCenter = trackBestAngle;
      trackBestDist = INVALID_DIST; trackBestAngle = trackingCenter;
      minCone = max(SERVO_MIN_ANGLE, trackingCenter - TRACK_WIDTH);
      maxCone = min(SERVO_MAX_ANGLE, trackingCenter + TRACK_WIDTH);
      trackSweepLeft = !trackSweepLeft;
      int nextTarget = trackSweepLeft ? minCone : maxCone;
      myServo.setSpeed(SERVO_SPEED_TRACK); myServo.startEaseTo(nextTarget);
    }
  } else {
    if (!myServo.isMoving()) {
      currentScanTarget = (currentScanTarget >= SERVO_MAX_ANGLE - 5) ? SERVO_MIN_ANGLE : SERVO_MAX_ANGLE;
      myServo.setSpeed(SERVO_SPEED_SCAN); myServo.startEaseTo(currentScanTarget);
    }
  }
}

void updatePeripherals(int finalDist, int currentServoAngle, unsigned long now) {
  if (finalDist > 0 && finalDist < MAX_WORLD_DIST) radarMap[currentServoAngle] = finalDist; 
  else radarMap[currentServoAngle] = 0;

  digitalWrite(PIN_LED_V, !isDangerActive && !isTrackingMode ? HIGH : LOW);
  digitalWrite(PIN_LED_Y, isTrackingMode && !isDangerActive ? HIGH : LOW);
  digitalWrite(PIN_LED_R, isDangerActive ? HIGH : LOW);
  digitalWrite(PIN_BUZZ, isDangerActive ? HIGH : LOW);

  if (now - lastDisplayUpdate > DISPLAY_UPDATE_MS) {
    if (isDangerActive) drawDanger(lastKnownDangerDist);
    else if (isTrackingMode) drawTracking(finalDist < LOCK_DIST ? finalDist : lastKnownDangerDist);
    else drawScanning(currentServoAngle);
    lastDisplayUpdate = now;
  }
}

// --- SETUP & LOOP ---
void setup() {
  Serial.begin(115200);
   
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_config = { .timeout_ms = 30000, .idle_core_mask = (1 << 1), .trigger_panic = true };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  pinMode(PIN_LED_V, OUTPUT); pinMode(PIN_LED_Y, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);

  Wire.begin(21, 22); display.begin(0x3C, true);
  lox.begin();

  if (myServo.attach(PIN_SERVO) != -1) {
    myServo.setEasingType(EASE_LINEAR);
    myServo.setSpeed(SERVO_SPEED_SCAN); myServo.startEaseTo(SERVO_MIN_ANGLE);
  }

  xTaskCreatePinnedToCore(TaskTelegram, "TelegramTask", 10000, NULL, 1, NULL, 0);
}

void loop() {
  esp_task_wdt_reset();

  if (bootPhase < 3) {
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE); display.setCursor(10, 20); 
    if (bootPhase == 0) display.print("WIFI CONNECTING...");
    else if (bootPhase == 1) display.print("WIFI OK! ONLINE");
    else if (bootPhase == 2) display.print("WIFI FAIL. OFFLINE");
    display.display();
    return;
  }

  unsigned long now = millis();
  int currentServoAngle = myServo.getCurrentAngle();
  
  int finalDist = readSensors();
  sharedDist = finalDist;
  sharedAngle = currentServoAngle;

  updateSystemState(finalDist, now);
  updateServo(finalDist, currentServoAngle);
  updatePeripherals(finalDist, currentServoAngle, now);

  delay(10);
}

// --- FUNZIONI DI DISEGNO DISPLAY ---
void drawScanning(int angle) {
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("SCANNING");
  
  display.drawCircle(SCAN_CENTER_X, SCAN_CENTER_Y, SCAN_RADIUS_OUTER, SH110X_WHITE); 
  display.drawCircle(SCAN_CENTER_X, SCAN_CENTER_Y, SCAN_RADIUS_INNER, SH110X_WHITE);
  
  for (int i = 0; i < RADAR_MAP_SIZE; i++) {
    if (radarMap[i] > 0) {
      float rad = i * PI / 180.0;
      int distPx = map(radarMap[i], 0, MAX_WORLD_DIST, 0, SCAN_RADIUS_OUTER);
      int px = SCAN_CENTER_X + distPx * cos(rad);
      int py = SCAN_CENTER_Y - distPx * sin(rad);
      
      if (radarMap[i] < LOCK_DIST) display.drawCircle(px, py, 1, SH110X_WHITE);
      else display.drawPixel(px, py, SH110X_WHITE);
    }
  }
  float lineRad = (180 - angle) * PI / 180.0;
  display.drawLine(SCAN_CENTER_X, SCAN_CENTER_Y, SCAN_CENTER_X + SCAN_RADIUS_OUTER * cos(lineRad), SCAN_CENTER_Y - SCAN_RADIUS_OUTER * sin(lineRad), SH110X_WHITE);
  display.display();
}

void drawTracking(int d) {
  display.clearDisplay(); 
  display.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE); 
  
  // Mirino
  display.drawLine(TRACK_CROSSHAIR_X, 10, TRACK_CROSSHAIR_X, OLED_HEIGHT - 10, SH110X_WHITE); 
  display.drawLine(30, TRACK_CROSSHAIR_Y, 98, TRACK_CROSSHAIR_Y, SH110X_WHITE);
  display.fillCircle(TRACK_CROSSHAIR_X, TRACK_CROSSHAIR_Y, TRACK_CROSSHAIR_R, SH110X_WHITE);
  
  // Testi
  display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(4, 4); display.print("TRACKING");
  display.setCursor(4, OLED_HEIGHT - 10); display.print("DST:"); display.print(d); display.print("cm");
  
  if (isTrackingMode) { display.setCursor(OLED_WIDTH - 38, 4); display.print("[LOCK]"); }
  display.display();
}

void drawDanger(int d) {
  display.clearDisplay(); 
  display.fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE);
  
  display.setTextColor(SH110X_BLACK);
  display.setTextSize(2); display.setCursor(OLED_WIDTH / 2 - 39, 15); display.print("DANGER");
  display.setTextSize(2); display.setCursor(OLED_WIDTH / 2 - 29, 40); display.print(d); display.print(" cm");
  
  display.setTextColor(SH110X_WHITE);
  display.display();
}