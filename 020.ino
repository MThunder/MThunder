#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <RTClib.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <time.h>

// ============================================================
// OLED
// ============================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE
);

// ============================================================
// RTC
// ============================================================

RTC_DS3231 rtc;

// ============================================================
// PINS
// ============================================================

#define ENC_A         D5
#define ENC_B         D6
#define ENC_PUSH      D7
#define BTN_BACK      D0

// ------------------------------------------------------------
// ВЫХОДЫ
// D8 = Нагрузка
// D4 = Зарядка
// D3 = Резерв 1
// ------------------------------------------------------------

#define MOSFET1_PIN   D8
#define MOSFET2_PIN   D4
#define MOSFET3_PIN   D3

// ============================================================
// BATTERY
// ============================================================

const float VOLTS_PER_RAW = 0.01810;

const int BATTERY_SAMPLES = 256;

const unsigned long BATTERY_UPDATE_INTERVAL = 500;

const float BATTERY_FILTER_ALPHA = 0.20;

float batteryVoltageRaw = 0.0;
float batteryVoltage = 0.0;

bool batteryInitialized = false;

unsigned long lastBatteryUpdate = 0;

// ------------------------------------------------------------
// Защита от глубокого разряда
// true после срабатывания нижнего порога.
// Сбрасывается только при включении зарядки.
// ------------------------------------------------------------

bool batteryCriticalActive = false;

// Предыдущее состояние линии зарядки.
// Нужен для определения именно момента её включения.
bool previousChargeState = false;

// ============================================================
// BATTERY SETTINGS
// ============================================================

float batteryFullVoltage = 14.60;
float batteryCriticalVoltage = 10.50;
float batteryCorrection = 0.00;

// ------------------------------------------------------------
// DELAY
// ------------------------------------------------------------

int batteryDelayMinutes = 40;

const int BATTERY_DELAY_MIN = 5;
const int BATTERY_DELAY_MAX = 120;
const int BATTERY_DELAY_STEP = 5;

// ------------------------------------------------------------
// VOLTAGE SETTINGS
// ------------------------------------------------------------

const float BATTERY_SETTING_STEP = 0.05;

const float FULL_VOLTAGE_MIN = 0.00;
const float FULL_VOLTAGE_MAX = 20.00;

const float CRITICAL_VOLTAGE_MIN = 0.00;
const float CRITICAL_VOLTAGE_MAX = 20.00;

const float CORRECTION_MIN = -2.00;
const float CORRECTION_MAX = 2.00;

// ============================================================
// ENCODER
// ============================================================

int8_t lastState = 0;
int8_t accumulator = 0;
long position = 0;

const int8_t transitionTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ============================================================
// PUSH BUTTON
// ============================================================

bool pushLast = HIGH;
unsigned long pushPressTime = 0;
bool pushLongDone = false;

// ============================================================
// BACK BUTTON
// ============================================================

bool backLast = HIGH;

// ============================================================
// MAIN OUTPUT SELECTION
// ============================================================

// 0 = Нагрузка
// 1 = Зарядка
// 2 = Резерв 1
//
// Резерв 2 пока оставляем только как будущий вариант:
// 3 = Резерв 2

int mainOutputSelection = 0;

bool mainOutputSelectMode = false;

unsigned long lastMainOutputActivity = 0;

const unsigned long MAIN_OUTPUT_TIMEOUT = 5000;

// ------------------------------------------------------------
// Резерв 1
// Состояние теперь физически читаем с MOSFET3_PIN.
// ------------------------------------------------------------

// bool reserve1State = false;

// ------------------------------------------------------------
// Резерв 2 — ПОКА НЕ ИСПОЛЬЗУЕТСЯ
// ------------------------------------------------------------

// bool reserve2State = false;

// ============================================================
// ТАЙМЕРЫ ВЫХОДОВ
// ============================================================

// 0 = Нагрузка
// 1 = Зарядка
// 2 = Резерв 1
//
// 3 = Резерв 2 — ПОКА НЕ ИСПОЛЬЗУЕТСЯ

struct OutputTimer {

  int onMinutes;
  int offMinutes;

  bool enabled;
};

OutputTimer outputTimers[3] = {

  // Нагрузка
  { 6 * 60 + 50, 23 * 60 + 10, false },

  // Зарядка
  { 0, 0, false },

  // Резерв 1
  { 0, 0, false }
};

// ------------------------------------------------------------
// Выбор таймера
// ------------------------------------------------------------

int timerPos = 0;

// 0 = ВКЛ
// 1 = ВЫКЛ
// 2 = Таймер

int timerEditField = 0;

// 0 = часы, 1 = минуты
int timerSubField = 0;

bool timerEditing = false;
bool timerValueEditing = false;

// Состояние мигания часов/минут при редактировании таймера
bool timerBlinkState = true;
unsigned long timerBlinkMillis = 0;

const int TIMER_STEP = 5;
const int TIMER_MINUTES_MAX = 23 * 60 + 55;

int lastTimerMinute = -1;
int lastTimerDay = -1;

// ============================================================
// WIFI
// ============================================================

String savedSSID;
String savedPassword;

bool wifiConnected = false;

// ============================================================
// WIFI SCAN
// ============================================================

String wifiNames[20];

int wifiCount = 0;
int wifiPos = 0;

// ============================================================
// WIFI PASSWORD
// ============================================================

String wifiPassword = "";

const char passwordChars[] =
  "abcdefghijklmnopqrstuvwxyz"
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "0123456789"
  "!@#$%^&*()-_=+[]{}.,;:";

const int PASSWORD_CHAR_COUNT =
  sizeof(passwordChars) - 1;

int passwordCharPos = 0;

// ============================================================
// SCREENS
// ============================================================

enum Screen {

  SCREEN_MAIN,

  SCREEN_MENU,

  SCREEN_BATTERY,
  SCREEN_OUTPUTS,
  SCREEN_SCHEDULE,
  SCREEN_CLOCK,

  SCREEN_WIFI,
  SCREEN_WIFI_SCAN,
  SCREEN_WIFI_PASSWORD,
  SCREEN_WIFI_CONNECTING,

  SCREEN_SETTINGS
};

Screen screen = SCREEN_MAIN;

// ============================================================
// MAIN MENU
// ============================================================

int menuPos = 0;

const char* menuItems[] = {

  "Батарея",
  "Выходы",
  "Таймеры",
  "Часы",
  "Wi-Fi",
  "Настройки"
};

const int MENU_COUNT = 6;

// ============================================================
// SETTINGS
// ============================================================

int settingsPos = 0;

bool settingsEditing = false;

const int SETTINGS_COUNT = 4;

const char* settingsItems[] = {

  "Заряженный",
  "Разряженный",
  "Кор.напряж.",
  "Задержка"
};

// ============================================================
// TIME
// ============================================================

const long GMT_OFFSET_SEC = 3 * 3600;
const int DST_OFFSET_SEC = 0;

// ============================================================
// LITTLEFS
// ============================================================

bool saveWiFiCredentials(
  const String& ssid,
  const String& password
) {

  File file = LittleFS.open("/wifi.txt", "w");

  if (!file) {

    Serial.println("ERROR: Cannot open /wifi.txt");

    return false;
  }

  file.println(ssid);
  file.println(password);

  file.close();

  Serial.println("WiFi credentials saved.");

  return true;
}

// ------------------------------------------------------------

bool loadWiFiCredentials() {

  if (!LittleFS.exists("/wifi.txt")) {

    Serial.println("No saved WiFi credentials.");

    return false;
  }

  File file = LittleFS.open("/wifi.txt", "r");

  if (!file) {

    Serial.println("ERROR: Cannot open /wifi.txt");

    return false;
  }

  savedSSID =
    file.readStringUntil('\n');

  savedPassword =
    file.readStringUntil('\n');

  file.close();

  savedSSID.trim();
  savedPassword.trim();

  if (savedSSID.length() == 0) {

    Serial.println("Saved WiFi file is empty.");

    return false;
  }

  Serial.println();
  Serial.println("Saved WiFi credentials found.");

  Serial.print("SSID: ");
  Serial.println(savedSSID);

  Serial.print("Password length: ");
  Serial.println(savedPassword.length());

  return true;
}

// ============================================================
// SAVE BATTERY SETTINGS
// ============================================================

void saveBatterySettings() {

  File file =
    LittleFS.open("/battery.txt", "w");

  if (!file) {

    Serial.println(
      "ERROR: Cannot open /battery.txt"
    );

    return;
  }

  file.println(
    String(batteryFullVoltage, 2)
  );

  file.println(
    String(batteryCriticalVoltage, 2)
  );

  file.println(
    String(batteryCorrection, 2)
  );

  file.println(
    batteryDelayMinutes
  );

  file.close();

  Serial.println(
    "Battery settings saved."
  );
}

// ============================================================
// LOAD BATTERY SETTINGS
// ============================================================

void loadBatterySettings() {

  if (!LittleFS.exists("/battery.txt")) {

    Serial.println(
      "No battery settings. Using defaults."
    );

    return;
  }

  File file =
    LittleFS.open("/battery.txt", "r");

  if (!file) {

    Serial.println(
      "ERROR: Cannot open /battery.txt"
    );

    return;
  }

  String s1 =
    file.readStringUntil('\n');

  String s2 =
    file.readStringUntil('\n');

  String s3 =
    file.readStringUntil('\n');

  String s4 =
    file.readStringUntil('\n');

  file.close();

  s1.trim();
  s2.trim();
  s3.trim();
  s4.trim();

  if (s1.length() > 0)
    batteryFullVoltage =
      s1.toFloat();

  if (s2.length() > 0)
    batteryCriticalVoltage =
      s2.toFloat();

  if (s3.length() > 0)
    batteryCorrection =
      s3.toFloat();

  if (s4.length() > 0)
    batteryDelayMinutes =
      s4.toInt();

  // ----------------------------------------------------------
  // Защита от мусора в файле
  // ----------------------------------------------------------

  if (
    batteryFullVoltage < FULL_VOLTAGE_MIN ||
    batteryFullVoltage > FULL_VOLTAGE_MAX
  )
    batteryFullVoltage = 14.60;

  if (
    batteryCriticalVoltage < CRITICAL_VOLTAGE_MIN ||
    batteryCriticalVoltage > CRITICAL_VOLTAGE_MAX
  )
    batteryCriticalVoltage = 10.50;

  if (
    batteryCorrection < CORRECTION_MIN ||
    batteryCorrection > CORRECTION_MAX
  )
    batteryCorrection = 0.00;

  if (
    batteryDelayMinutes < BATTERY_DELAY_MIN ||
    batteryDelayMinutes > BATTERY_DELAY_MAX
  )
    batteryDelayMinutes = 40;

  Serial.println();
  Serial.println("Battery settings loaded.");

  Serial.print("Charged: ");
  Serial.println(batteryFullVoltage, 2);

  Serial.print("Critical: ");
  Serial.println(batteryCriticalVoltage, 2);

  Serial.print("Correction: ");
  Serial.println(batteryCorrection, 2);

  Serial.print("Delay: ");
  Serial.print(batteryDelayMinutes);
  Serial.println(" min");
}

// ============================================================
// SAVE TIMER SETTINGS
// ============================================================

void saveTimerSettings() {

  File file =
    LittleFS.open("/timers.txt", "w");

  if (!file) {

    Serial.println(
      "ERROR: Cannot open /timers.txt"
    );

    return;
  }

  for (int i = 0; i < 3; i++) {

    file.print(
      outputTimers[i].onMinutes
    );

    file.print(";");

    file.print(
      outputTimers[i].offMinutes
    );

    file.print(";");

    file.println(
      outputTimers[i].enabled ? 1 : 0
    );
  }

  file.close();

  Serial.println(
    "Timer settings saved."
  );
}

// ============================================================
// LOAD TIMER SETTINGS
// ============================================================

void loadTimerSettings() {

  if (!LittleFS.exists("/timers.txt")) {

    Serial.println(
      "No timer settings. Using defaults."
    );

    return;
  }

  File file =
    LittleFS.open("/timers.txt", "r");

  if (!file) {

    Serial.println(
      "ERROR: Cannot open /timers.txt"
    );

    return;
  }

  for (int i = 0; i < 3; i++) {

    if (!file.available())
      break;

    String line =
      file.readStringUntil('\n');

    line.trim();

    int p1 =
      line.indexOf(';');

    int p2 =
      line.indexOf(';', p1 + 1);

    if (
      p1 <= 0 ||
      p2 <= p1
    )
      continue;

    int onTime =
      line.substring(
        0,
        p1
      ).toInt();

    int offTime =
      line.substring(
        p1 + 1,
        p2
      ).toInt();

    int enabled =
      line.substring(
        p2 + 1
      ).toInt();

    if (
      onTime >= 0 &&
      onTime <= TIMER_MINUTES_MAX
    )
      outputTimers[i].onMinutes =
        onTime;

    if (
      offTime >= 0 &&
      offTime <= TIMER_MINUTES_MAX
    )
      outputTimers[i].offMinutes =
        offTime;

    outputTimers[i].enabled =
      (enabled != 0);
  }

  file.close();

  Serial.println(
    "Timer settings loaded."
  );
}

// ============================================================
// CONNECT SAVED WIFI
// ============================================================

bool connectSavedWiFi() {

  if (savedSSID.length() == 0)
    return false;

  Serial.println();
  Serial.println("================================");
  Serial.println("WIFI AUTO CONNECT");
  Serial.println("================================");

  Serial.print("SSID: ");
  Serial.println(savedSSID);

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    savedSSID.c_str(),
    savedPassword.c_str()
  );

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 15000
  ) {

    delay(250);

    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    wifiConnected = true;

    Serial.println("CONNECTED!");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    return true;
  }

  wifiConnected = false;

  Serial.println("WiFi connection failed.");

  return false;
}

// ============================================================
// NTP → RTC
// ============================================================

void syncRTCFromNTP() {

  if (WiFi.status() != WL_CONNECTED)
    return;

  Serial.println();
  Serial.println("=== NTP SYNC ===");

  configTime(
    GMT_OFFSET_SEC,
    DST_OFFSET_SEC,
    "pool.ntp.org",
    "time.nist.gov",
    "time.google.com"
  );

  time_t now = time(nullptr);

  unsigned long start = millis();

  while (
    now < 100000 &&
    millis() - start < 10000
  ) {

    delay(200);

    Serial.print(".");

    now = time(nullptr);
  }

  Serial.println();

  if (now < 100000) {

    Serial.println("NTP: FAILED");

    return;
  }

  struct tm* tmNow =
    localtime(&now);

  DateTime dt(

    tmNow->tm_year + 1900,
    tmNow->tm_mon + 1,
    tmNow->tm_mday,

    tmNow->tm_hour,
    tmNow->tm_min,
    tmNow->tm_sec
  );

  Serial.printf(

    "NTP: %02d.%02d.%04d  %02d:%02d:%02d\n",

    dt.day(),
    dt.month(),
    dt.year(),

    dt.hour(),
    dt.minute(),
    dt.second()
  );

  rtc.adjust(dt);

  Serial.println("DS3231: UPDATED");
  Serial.println("NTP -> DS3231: OK");
}

// ============================================================
// WIFI SCAN
// ============================================================

void scanWiFi() {

  Serial.println();
  Serial.println("================================");
  Serial.println("WIFI SCAN");
  Serial.println("================================");

  WiFi.mode(WIFI_STA);

  int count =
    WiFi.scanNetworks();

  wifiCount = 0;

  if (count <= 0) {

    Serial.println("No networks found.");

    return;
  }

  for (
    int i = 0;
    i < count && wifiCount < 20;
    i++
  ) {

    wifiNames[wifiCount] =
      WiFi.SSID(i);

    Serial.print(wifiCount);
    Serial.print(": ");

    Serial.print(wifiNames[wifiCount]);

    Serial.print("  ");

    Serial.print(WiFi.RSSI(i));

    Serial.println(" dBm");

    wifiCount++;
  }

  wifiPos = 0;

  WiFi.scanDelete();

  Serial.print("Found: ");
  Serial.println(wifiCount);
}

// ============================================================
// CONNECT NEW WIFI
// ============================================================

void connectNewWiFi() {

  Serial.println();
  Serial.println("================================");
  Serial.println("WIFI CONNECT NEW");
  Serial.println("================================");

  Serial.print("SSID: ");
  Serial.println(wifiNames[wifiPos]);

  WiFi.disconnect();

  delay(300);

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    wifiNames[wifiPos].c_str(),
    wifiPassword.c_str()
  );

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 15000
  ) {

    delay(250);

    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("CONNECTED!");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    savedSSID =
      wifiNames[wifiPos];

    savedPassword =
      wifiPassword;

    saveWiFiCredentials(
      savedSSID,
      savedPassword
    );

    wifiConnected = true;

    syncRTCFromNTP();

    screen = SCREEN_WIFI;
  }

  else {

    Serial.println("CONNECTION FAILED");

    wifiPassword = "";

    screen = SCREEN_WIFI;
  }

  drawScreen();
}

// ============================================================
// BATTERY MEASUREMENT
// ============================================================

void updateBattery() {

  if (
    millis() - lastBatteryUpdate <
    BATTERY_UPDATE_INTERVAL
  )
    return;

  lastBatteryUpdate = millis();

  uint32_t sum = 0;

  for (
    int i = 0;
    i < BATTERY_SAMPLES;
    i++
  ) {

    sum += analogRead(A0);

    if ((i & 31) == 31)
      yield();
  }

  float raw =
    (float)sum /
    BATTERY_SAMPLES;

  batteryVoltageRaw =
    raw * VOLTS_PER_RAW;

  float correctedVoltage =
    batteryVoltageRaw +
    batteryCorrection;

  if (!batteryInitialized) {

    batteryVoltage =
      correctedVoltage;

    batteryInitialized = true;
  }

  else {

    batteryVoltage =
      batteryVoltage +
      BATTERY_FILTER_ALPHA *
      (
        correctedVoltage -
        batteryVoltage
      );
  }

  // ----------------------------------------------------------
  // ЗАЩИТА ОТ ГЛУБОКОГО РАЗРЯДА
  // ----------------------------------------------------------

  bool chargeState =
    digitalRead(MOSFET2_PIN);

  // Зарядка только что включилась:
  // снимаем блокировку после предыдущего
  // критического разряда.
  if (
    chargeState &&
    !previousChargeState
  ) {

    if (batteryCriticalActive) {

      batteryCriticalActive = false;

      Serial.println(
        "CHARGER ON: BATTERY CRITICAL RESET"
      );
    }
  }

  previousChargeState =
    chargeState;

  // Проверяем только когда нагрузка действительно
  // работает от аккумулятора.
  if (
    digitalRead(MOSFET1_PIN) &&
    batteryVoltage <= batteryCriticalVoltage
  ) {

    batteryCriticalActive = true;

    // Переключаем нагрузку на СЕТЬ.
    digitalWrite(
      MOSFET1_PIN,
      LOW
    );

    Serial.println();
    Serial.println(
      "!!! BATTERY CRITICAL !!!"
    );

    Serial.print(
      "Voltage: "
    );

    Serial.print(
      batteryVoltage,
      2
    );

    Serial.println(
      " V"
    );

    Serial.println(
      "LOAD -> GRID"
    );
  }
}

// ============================================================
// WIFI SIGNAL LEVEL
// ============================================================

int getWiFiSignalLevel() {

  if (
    WiFi.status() != WL_CONNECTED
  )
    return 0;

  int rssi =
    WiFi.RSSI();

  if (rssi >= -55)
    return 4;

  if (rssi >= -67)
    return 3;

  if (rssi >= -75)
    return 2;

  return 1;
}

// ============================================================
// WIFI SIGNAL ICON
// ============================================================

void drawWiFiIcon(
  int x,
  int y,
  int level
) {

  oled.drawVLine(
    x + 1,
    y - 8,
    8
  );

  oled.drawLine(
    x + 1, y - 5,
    x - 2, y - 8
  );

  oled.drawLine(
    x + 1, y - 5,
    x + 4, y - 8
  );

  const int width = 2;
  const int gap   = 1;

  for (int i = 0; i < 4; i++) {

    if (i < level) {

      int height = 3 + i * 2;

      oled.drawBox(
        x + 7 + i * (width + gap),
        y - height,
        width,
        height
      );
    }
  }
}

// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {

  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);

  int currentState =
    (A << 1) | B;

  if (currentState == lastState)
    return;

  int index =
    (lastState << 2) | currentState;

  int8_t movement =
    transitionTable[index];

  if (movement != 0) {

    accumulator += movement;

    // ========================================================
    // НЕ ТРОГАЕМ.
    // Это рабочая чувствительность энкодера.
    // ========================================================

    if (accumulator >= 4) {

      position++;

      accumulator = 0;

      handleEncoderRight();
    }

    if (accumulator <= -4) {

      position--;

      accumulator = 0;

      handleEncoderLeft();
    }
  }

  lastState = currentState;
}

// ============================================================
// TIMER HELPERS
// ============================================================

void resetTimerEditor() {

  timerEditing = false;
  timerValueEditing = false;
  timerEditField = 0;
  timerSubField = 0;
}

void changeTimerValue(int &value, int direction) {

  if (timerSubField == 0) {

    int hours = value / 60 + direction;

    if (hours > 23) hours = 0;
    if (hours < 0) hours = 23;

    value = hours * 60 + value % 60;
  }

  else {

    int minutes =
      value % 60 +
      direction * TIMER_STEP;

    if (minutes >= 60) minutes = 0;
    if (minutes < 0) minutes = 55;

    value =
      (value / 60) * 60 +
      minutes;
  }
}

void changeTimer(int direction) {

  if (!timerEditing) {

    timerPos += direction;

    if (timerPos >= 3) timerPos = 0;
    if (timerPos < 0) timerPos = 2;

    return;
  }

  if (!timerValueEditing) {

    timerEditField += direction;

    if (timerEditField > 2) timerEditField = 0;
    if (timerEditField < 0) timerEditField = 2;

    return;
  }

  if (timerEditField == 0)
    changeTimerValue(
      outputTimers[timerPos].onMinutes,
      direction
    );

  else if (timerEditField == 1)
    changeTimerValue(
      outputTimers[timerPos].offMinutes,
      direction
    );

  else
    outputTimers[timerPos].enabled =
      !outputTimers[timerPos].enabled;
}

// ============================================================
// ENCODER RIGHT
// ============================================================

void handleEncoderRight() {

  // ----------------------------------------------------------
  // ГЛАВНЫЙ ЭКРАН
  // ----------------------------------------------------------

  if (screen == SCREEN_MAIN) {

    if (!mainOutputSelectMode) {

      mainOutputSelectMode = true;
      mainOutputSelection = 0;

    } else {

      mainOutputSelection++;

      if (mainOutputSelection > 2)
        mainOutputSelection = 0;
    }

    lastMainOutputActivity = millis();

    Serial.print("MAIN OUTPUT SELECT: ");

    if (mainOutputSelection == 0)
      Serial.println("Нагрузка");

    else if (mainOutputSelection == 1)
      Serial.println("Зарядка");

    else
      Serial.println("Резерв 1");
  }

  // ----------------------------------------------------------
  // ГЛАВНОЕ МЕНЮ
  // ----------------------------------------------------------

  else if (screen == SCREEN_MENU) {

    menuPos++;

    if (menuPos >= MENU_COUNT)
      menuPos = 0;
  }

  // ----------------------------------------------------------
  // WIFI SCAN
  // ----------------------------------------------------------

  else if (
    screen == SCREEN_WIFI_SCAN
  ) {

    if (wifiCount > 0) {

      wifiPos++;

      if (wifiPos >= wifiCount)
        wifiPos = 0;
    }
  }

  // ----------------------------------------------------------
  // WIFI PASSWORD
  // ----------------------------------------------------------

  else if (
    screen == SCREEN_WIFI_PASSWORD
  ) {

    passwordCharPos++;

    if (
      passwordCharPos >=
      PASSWORD_CHAR_COUNT
    )
      passwordCharPos = 0;
  }

  // ----------------------------------------------------------
  // ТАЙМЕРЫ
  // ----------------------------------------------------------

  else if (screen == SCREEN_SCHEDULE) {

    changeTimer(1);
  }

  // ----------------------------------------------------------
  // SETTINGS
  // ----------------------------------------------------------

  else if (
    screen == SCREEN_SETTINGS
  ) {

    if (!settingsEditing) {

      settingsPos++;

      if (
        settingsPos >= SETTINGS_COUNT
      )
        settingsPos = 0;
    }

    else {

      if (settingsPos == 0) {

        batteryFullVoltage +=
          BATTERY_SETTING_STEP;

        if (
          batteryFullVoltage >
          FULL_VOLTAGE_MAX
        )
          batteryFullVoltage =
            FULL_VOLTAGE_MAX;
      }

      else if (settingsPos == 1) {

        batteryCriticalVoltage +=
          BATTERY_SETTING_STEP;

        if (
          batteryCriticalVoltage >
          CRITICAL_VOLTAGE_MAX
        )
          batteryCriticalVoltage =
            CRITICAL_VOLTAGE_MAX;
      }

      else if (settingsPos == 2) {

        batteryCorrection +=
          BATTERY_SETTING_STEP;

        if (
          batteryCorrection >
          CORRECTION_MAX
        )
          batteryCorrection =
            CORRECTION_MAX;
      }

      else if (settingsPos == 3) {

        batteryDelayMinutes +=
          BATTERY_DELAY_STEP;

        if (
          batteryDelayMinutes >
          BATTERY_DELAY_MAX
        )
          batteryDelayMinutes =
            BATTERY_DELAY_MAX;
      }
    }
  }

  drawScreen();
}

// ============================================================
// ENCODER LEFT
// ============================================================

void handleEncoderLeft() {

  // ----------------------------------------------------------
  // ГЛАВНЫЙ ЭКРАН
  // ----------------------------------------------------------

  if (screen == SCREEN_MAIN) {

    if (!mainOutputSelectMode) {

      mainOutputSelectMode = true;
      mainOutputSelection = 0;

    } else {

      mainOutputSelection--;

      if (mainOutputSelection < 0)
        mainOutputSelection = 2;
    }

    lastMainOutputActivity = millis();

    Serial.print("MAIN OUTPUT SELECT: ");

    if (mainOutputSelection == 0)
      Serial.println("Нагрузка");

    else if (mainOutputSelection == 1)
      Serial.println("Зарядка");

    else
      Serial.println("Резерв 1");
  }

  // ----------------------------------------------------------
  // ГЛАВНОЕ МЕНЮ
  // ----------------------------------------------------------

  else if (screen == SCREEN_MENU) {

    menuPos--;

    if (menuPos < 0)
      menuPos = MENU_COUNT - 1;
  }

  // ----------------------------------------------------------
  // WIFI SCAN
  // ----------------------------------------------------------

  else if (
    screen == SCREEN_WIFI_SCAN
  ) {

    if (wifiCount > 0) {

      wifiPos--;

      if (wifiPos < 0)
        wifiPos =
          wifiCount - 1;
    }
  }

  // ----------------------------------------------------------
  // WIFI PASSWORD
  // ----------------------------------------------------------

  else if (
    screen == SCREEN_WIFI_PASSWORD
  ) {

    passwordCharPos--;

    if (passwordCharPos < 0)
      passwordCharPos =
        PASSWORD_CHAR_COUNT - 1;
  }

  // ----------------------------------------------------------
  // ТАЙМЕРЫ
  // ----------------------------------------------------------

  else if (screen == SCREEN_SCHEDULE) {

    changeTimer(-1);
  }

  // ----------------------------------------------------------
  // SETTINGS
  // ----------------------------------------------------------

  else if (
    screen == SCREEN_SETTINGS
  ) {

    if (!settingsEditing) {

      settingsPos--;

      if (settingsPos < 0)
        settingsPos =
          SETTINGS_COUNT - 1;
    }

    else {

      if (settingsPos == 0) {

        batteryFullVoltage -=
          BATTERY_SETTING_STEP;

        if (
          batteryFullVoltage <
          FULL_VOLTAGE_MIN
        )
          batteryFullVoltage =
            FULL_VOLTAGE_MIN;
      }

      else if (settingsPos == 1) {

        batteryCriticalVoltage -=
          BATTERY_SETTING_STEP;

        if (
          batteryCriticalVoltage <
          CRITICAL_VOLTAGE_MIN
        )
          batteryCriticalVoltage =
            CRITICAL_VOLTAGE_MIN;
      }

      else if (settingsPos == 2) {

        batteryCorrection -=
          BATTERY_SETTING_STEP;

        if (
          batteryCorrection <
          CORRECTION_MIN
        )
          batteryCorrection =
            CORRECTION_MIN;
      }

      else if (settingsPos == 3) {

        batteryDelayMinutes -=
          BATTERY_DELAY_STEP;

        if (
          batteryDelayMinutes <
          BATTERY_DELAY_MIN
        )
          batteryDelayMinutes =
            BATTERY_DELAY_MIN;
      }
    }
  }

  drawScreen();
}

// ============================================================
// PUSH BUTTON
// ============================================================

void updatePush() {

  bool state =
    digitalRead(ENC_PUSH);

  // ----------------------------------------------------------
  // Нажатие
  // ----------------------------------------------------------

  if (
    pushLast == HIGH &&
    state == LOW
  ) {

    pushPressTime =
      millis();

    pushLongDone = false;
  }

  // ----------------------------------------------------------
  // Долгое нажатие
  // ----------------------------------------------------------

  if (
    state == LOW &&
    !pushLongDone &&
    millis() - pushPressTime >= 1000
  ) {

    pushLongDone = true;

    Serial.println(
      ">>> PUSH LONG PRESS <<<"
    );

    // ========================================================
    // ГЛАВНЫЙ ЭКРАН
    // ========================================================

    if (
      screen == SCREEN_MAIN &&
      mainOutputSelectMode
    ) {

      // ------------------------------------------------------
      // Нагрузка — D8
      // ------------------------------------------------------

      if (mainOutputSelection == 0) {

        bool stateNow =
          digitalRead(MOSFET1_PIN);

        digitalWrite(
          MOSFET1_PIN,
          !stateNow
        );

        Serial.print(
          "Нагрузка: "
        );

        Serial.println(
          !stateNow ? "ВКЛ" : "ВЫКЛ"
        );
      }

      // ------------------------------------------------------
      // Зарядка — D4
      // ------------------------------------------------------

      else if (
        mainOutputSelection == 1
      ) {

        bool stateNow =
          digitalRead(MOSFET2_PIN);

        digitalWrite(
          MOSFET2_PIN,
          !stateNow
        );

        Serial.print(
          "Зарядка: "
        );

        Serial.println(
          !stateNow ? "ВКЛ" : "ВЫКЛ"
        );
      }

      // ------------------------------------------------------
      // Резерв 1 — D3
      // ------------------------------------------------------

      else if (
        mainOutputSelection == 2
      ) {

        bool stateNow =
          digitalRead(MOSFET3_PIN);

        digitalWrite(
          MOSFET3_PIN,
          !stateNow
        );

        Serial.print(
          "Резерв 1: "
        );

        Serial.println(
          !stateNow ? "ВКЛ" : "ВЫКЛ"
        );
      }

      lastMainOutputActivity =
        millis();

      drawScreen();
    }
  }

  // ----------------------------------------------------------
  // Отпускание
  // ----------------------------------------------------------

  if (
    pushLast == LOW &&
    state == HIGH
  ) {

    unsigned long duration =
      millis() - pushPressTime;

    // --------------------------------------------------------
    // Короткое нажатие
    // --------------------------------------------------------

    if (!pushLongDone) {

      Serial.print("SHORT PRESS  ");
      Serial.print(duration);
      Serial.println(" ms");

      handlePush();
    }

    // --------------------------------------------------------
    // Долгое нажатие
    // --------------------------------------------------------

    else {

      Serial.print(
        "RELEASE AFTER LONG PRESS  "
      );

      Serial.print(duration);

      Serial.println(" ms");

      if (
        screen ==
        SCREEN_WIFI_PASSWORD
      ) {

        Serial.println();
        Serial.println(
          "PASSWORD FINISHED"
        );

        screen =
          SCREEN_WIFI_CONNECTING;

        drawScreen();

        delay(300);

        connectNewWiFi();
      }
    }
  }

  pushLast = state;
}

// ============================================================
// PUSH ACTION
// ============================================================

void handlePush() {

  // ----------------------------------------------------------
  // MAIN
  // ----------------------------------------------------------

  if (screen == SCREEN_MAIN) {

    if (mainOutputSelectMode) {

      lastMainOutputActivity =
        millis();

      drawScreen();

      return;
    }

    screen =
      SCREEN_MENU;

    menuPos = 0;

    drawScreen();

    return;
  }

  // ----------------------------------------------------------
  // MAIN MENU
  // ----------------------------------------------------------

  if (screen == SCREEN_MENU) {

    switch (menuPos) {

      case 0:
        screen =
          SCREEN_BATTERY;
        break;

      case 1:
        screen =
          SCREEN_OUTPUTS;
        break;

      case 2:

        screen =
          SCREEN_SCHEDULE;

        timerPos = 0;
        resetTimerEditor();

        break;

      case 3:
        screen =
          SCREEN_CLOCK;
        break;

      case 4:
        screen =
          SCREEN_WIFI;
        break;

      case 5:

        screen =
          SCREEN_SETTINGS;

        settingsPos = 0;
        settingsEditing = false;

        break;
    }

    drawScreen();

    return;
  }

  // ----------------------------------------------------------
  // ТАЙМЕРЫ
  // ----------------------------------------------------------

  if (screen == SCREEN_SCHEDULE) {

    if (!timerEditing) {

      timerEditing = true;
      timerEditField = 0;
      timerSubField = 0;
      timerValueEditing = false;

      Serial.print("EDIT TIMER: ");
      Serial.println(
        timerPos == 0 ? "Нагрузка" :
        timerPos == 1 ? "Зарядка" :
        "Резерв 1"
      );
    }

    else if (!timerValueEditing) {

      timerValueEditing = true;
      timerSubField = 0;

      if (timerEditField == 2)
        Serial.println("EDIT TIMER STATE");
      else
        Serial.println("EDIT HOURS");
    }

    else if (timerEditField < 2) {

      if (timerSubField == 0) {

        timerSubField = 1;
        Serial.println("EDIT MINUTES");
      }

      else {

        timerValueEditing = false;
        timerSubField = 0;
        saveTimerSettings();
        Serial.println("TIME EDIT FINISHED");
      }
    }

    else {

      timerValueEditing = false;
      timerBlinkState = true;
      timerBlinkMillis = millis();
      saveTimerSettings();

      Serial.print("Timer: ");
      Serial.println(
        outputTimers[timerPos].enabled
          ? "ON"
          : "OFF"
      );
    }

    drawScreen();
    return;
  }

  // ----------------------------------------------------------
  // SETTINGS
  // ----------------------------------------------------------

  if (
    screen ==
    SCREEN_SETTINGS
  ) {

    if (!settingsEditing) {

      settingsEditing = true;

      Serial.print(
        "EDIT SETTING: "
      );

      Serial.println(
        settingsItems[settingsPos]
      );
    }

    else {

      settingsEditing = false;

      saveBatterySettings();

      Serial.println(
        "SETTING EDIT FINISHED"
      );
    }

    drawScreen();

    return;
  }

  // ----------------------------------------------------------
  // WIFI SCREEN
  // ----------------------------------------------------------

  if (screen == SCREEN_WIFI) {

    Serial.println();
    Serial.println(
      "Starting WiFi scan..."
    );

    screen =
      SCREEN_WIFI_SCAN;

    scanWiFi();

    drawScreen();

    return;
  }

  // ----------------------------------------------------------
  // WIFI NETWORK SELECT
  // ----------------------------------------------------------

  if (
    screen ==
    SCREEN_WIFI_SCAN
  ) {

    if (wifiCount == 0)
      return;

    wifiPassword = "";

    passwordCharPos = 0;

    screen =
      SCREEN_WIFI_PASSWORD;

    drawScreen();

    return;
  }

  // ----------------------------------------------------------
  // WIFI PASSWORD
  // ----------------------------------------------------------

  if (
    screen ==
    SCREEN_WIFI_PASSWORD
  ) {

    wifiPassword +=
      passwordChars[passwordCharPos];

    Serial.print("Password: ");

    for (
      unsigned int i = 0;
      i < wifiPassword.length();
      i++
    ) {

      Serial.print("*");
    }

    Serial.println();

    drawScreen();

    return;
  }
}

// ============================================================
// BACK BUTTON
// ============================================================

void updateBack() {

  bool state =
    digitalRead(BTN_BACK);

  if (
    backLast == HIGH &&
    state == LOW
  ) {

    // --------------------------------------------------------
    // Если на главном экране активен выбор выходов —
    // просто убираем выбор.
    // --------------------------------------------------------

    if (
      screen == SCREEN_MAIN &&
      mainOutputSelectMode
    ) {

      mainOutputSelectMode = false;

      drawScreen();
    }

    // --------------------------------------------------------
    // Если редактируем настройку —
    // сначала просто заканчиваем редактирование.
    // --------------------------------------------------------

    else if (
      screen == SCREEN_SETTINGS &&
      settingsEditing
    ) {

      settingsEditing = false;

      saveBatterySettings();

      drawScreen();
    }

    // --------------------------------------------------------
    // Из редактора таймера —
    // обратно в список таймеров.
    // --------------------------------------------------------

    else if (
      screen == SCREEN_SCHEDULE &&
      timerEditing
    ) {

      resetTimerEditor();
      drawScreen();
    }

    // --------------------------------------------------------
    // Из списка таймеров —
    // обратно в главное меню.
    // --------------------------------------------------------

    else if (
      screen == SCREEN_SCHEDULE &&
      !timerEditing
    ) {

      screen =
        SCREEN_MENU;

      drawScreen();
    }

    // --------------------------------------------------------
    // Из любого экрана второго уровня
    // возвращаемся в корневое меню.
    // --------------------------------------------------------

    else if (
      screen != SCREEN_MAIN &&
      screen != SCREEN_MENU
    ) {

      screen =
        SCREEN_MENU;

      wifiPassword = "";

      drawScreen();
    }

    // --------------------------------------------------------
    // Из корневого меню — на главный экран.
    // --------------------------------------------------------

    else if (
      screen == SCREEN_MENU
    ) {

      screen =
        SCREEN_MAIN;

      drawScreen();
    }
  }

  backLast = state;
}

// ============================================================
// DRAW MAIN
// ============================================================

void drawMain() {

  DateTime now = rtc.now();

  // ----------------------------------------------------------
  // НАПРЯЖЕНИЕ
  // ----------------------------------------------------------

  oled.setFont(
    u8g2_font_10x20_t_cyrillic
  );

  char voltageText[20];

  snprintf(
    voltageText,
    sizeof(voltageText),
    "%.2fV",
    batteryVoltage
  );

  if (
    batteryVoltage > batteryCriticalVoltage ||
    (millis() / 500) % 2 == 0
  ) {
    oled.drawStr(0, 18, voltageText);
  }

  // ----------------------------------------------------------
  // ВРЕМЯ
  // ----------------------------------------------------------

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  char timeText[10];

  snprintf(
    timeText,
    sizeof(timeText),
    "%02d:%02d",
    now.hour(),
    now.minute()
  );

  oled.drawStr(
    63,
    12,
    timeText
  );

  // ----------------------------------------------------------
  // Wi-Fi
  // ----------------------------------------------------------

  drawWiFiIcon(
    104,
    11,
    getWiFiSignalLevel()
  );

  // ----------------------------------------------------------
  // СОСТОЯНИЯ ВЫХОДОВ
  // ----------------------------------------------------------

  bool loadState =
    digitalRead(MOSFET1_PIN);

  bool chargeState =
    digitalRead(MOSFET2_PIN);

  bool reserve1State =
    digitalRead(MOSFET3_PIN);

  oled.setFont(
    u8g2_font_6x12_t_cyrillic
  );

  // ----------------------------------------------------------
  // Координаты строк
  // ----------------------------------------------------------

  const int rowY[3] = {
    32,
    46,
    60
  };

  const char* labels[3] = {

    "Нагрузка",
    "Зарядка",
    "Резерв 1"
  };

  const char* statuses[3] = {

    loadState
      ? "АККУМ"
      : "СЕТЬ",

    chargeState
      ? "ВКЛ"
      : "ВЫКЛ",

    reserve1State
      ? "ВКЛ"
      : "ВЫКЛ"
  };

  // ----------------------------------------------------------
  // Рисуем строки
  // ----------------------------------------------------------

  for (int i = 0; i < 3; i++) {

    bool selected =
      mainOutputSelectMode &&
      mainOutputSelection == i;

    if (selected) {

      oled.drawBox(
        0,
        rowY[i] - 10,
        128,
        12
      );

      oled.setDrawColor(0);
    }

    // --------------------------------------------------------
    // Название
    // --------------------------------------------------------

    oled.drawUTF8(
      0,
      rowY[i],
      labels[i]
    );

    // --------------------------------------------------------
    // Статус
    // --------------------------------------------------------

    oled.drawUTF8(
      70,
      rowY[i],
      statuses[i]
    );

    // --------------------------------------------------------
    // Кружок таймера
    // --------------------------------------------------------

    int circleX = 120;
    int circleY = rowY[i] - 4;

    if (outputTimers[i].enabled) {

      oled.drawDisc(
        circleX,
        circleY,
        4
      );

    } else {

      oled.drawCircle(
        circleX,
        circleY,
        4
      );
    }

    // --------------------------------------------------------
    // Возвращаем обычный цвет
    // --------------------------------------------------------

    if (selected) {

      oled.setDrawColor(1);
    }
  }
}

// ============================================================
// DRAW MENU
// ============================================================

void drawMenu() {

  oled.setFont(
    u8g2_font_unifont_t_cyrillic
  );

  int first =
    menuPos - 2;

  if (first < 0)
    first = 0;

  if (
    first >
    MENU_COUNT - 4
  )
    first =
      MENU_COUNT - 4;

  for (
    int i = 0;
    i < 4;
    i++
  ) {

    int item =
      first + i;

    if (
      item >=
      MENU_COUNT
    )
      break;

    int y =
      14 + i * 14;

    if (item == menuPos) {

      oled.drawBox(
        0,
        y - 11,
        128,
        14
      );

      oled.setDrawColor(0);

      oled.drawUTF8(
        4,
        y,
        menuItems[item]
      );

      oled.setDrawColor(1);
    }

    else {

      oled.drawUTF8(
        4,
        y,
        menuItems[item]
      );
    }
  }
}

// ============================================================
// DRAW BATTERY
// ============================================================

void drawBattery() {

  oled.setFont(
    u8g2_font_unifont_t_cyrillic
  );

  oled.drawUTF8(
    0,
    13,
    "БАТАРЕЯ"
  );

  oled.setFont(
    u8g2_font_10x20_t_cyrillic
  );

  char voltageText[20];

  snprintf(
    voltageText,
    sizeof(voltageText),
    "%.2fV",
    batteryVoltage
  );

  oled.drawStr(
    25,
    39,
    voltageText
  );

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  char correctionText[30];

  snprintf(
    correctionText,
    sizeof(correctionText),
    "Корр.: %+.2fV",
    batteryCorrection
  );

  oled.drawUTF8(
    0,
    58,
    correctionText
  );
}

// ============================================================
// DRAW OUTPUTS
// ============================================================

void drawOutputs() {

  oled.setFont(
    u8g2_font_unifont_t_cyrillic
  );

  oled.drawUTF8(
    0,
    13,
    "ВЫХОДЫ"
  );

  oled.drawUTF8(
    0,
    29,
    "Нагрузка"
  );

  oled.drawUTF8(
    85,
    29,
    digitalRead(MOSFET1_PIN)
      ? "ВКЛ"
      : "ВЫКЛ"
  );

  oled.drawUTF8(
    0,
    44,
    "Зарядка"
  );

  oled.drawUTF8(
    85,
    44,
    digitalRead(MOSFET2_PIN)
      ? "ВКЛ"
      : "ВЫКЛ"
  );

  oled.drawUTF8(
    0,
    59,
    "Резерв 1"
  );

  oled.drawUTF8(
    85,
    59,
    digitalRead(MOSFET3_PIN)
      ? "ВКЛ"
      : "ВЫКЛ"
  );

  // ----------------------------------------------------------
  // Резерв 2 — пока отключён из интерфейса.
  // ----------------------------------------------------------
}

// ============================================================
// DRAW TIMERS
// ============================================================

void drawSchedule() {

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  if (!timerEditing) {

    const char* items[3] = {
      "Т.Нагрузки",
      "Т.Зарядки",
      "Т.Резерва 1"
    };

    for (int i = 0; i < 3; i++) {

      int y =
        13 + i * 15;

      if (i == timerPos) {

        oled.drawBox(
          0,
          y - 10,
          128,
          12
        );

        oled.setDrawColor(0);
      }

      oled.drawUTF8(
        3,
        y,
        items[i]
      );

      if (i == timerPos)
        oled.setDrawColor(1);
    }

    return;
  }

  const char* title =
    timerPos == 0 ? "Т.Нагрузки" :
    timerPos == 1 ? "Т.Зарядки" :
    "Т.Резерва 1";

  oled.drawUTF8(
    0,
    12,
    title
  );

  char onText[24];
  char offText[24];
  char stateText[24];

  snprintf(
    onText,
    sizeof(onText),
    "ВКЛ:  %02d:%02d",
    outputTimers[timerPos].onMinutes / 60,
    outputTimers[timerPos].onMinutes % 60
  );

  snprintf(
    offText,
    sizeof(offText),
    "ВЫКЛ: %02d:%02d",
    outputTimers[timerPos].offMinutes / 60,
    outputTimers[timerPos].offMinutes % 60
  );

  snprintf(
    stateText,
    sizeof(stateText),
    "Таймер: %s",
    outputTimers[timerPos].enabled
      ? "ВКЛ"
      : "ВЫКЛ"
  );

  const char* lines[3] = {
    onText,
    offText,
    stateText
  };

  const int y[3] = {
    27,
    42,
    57
  };

  for (int i = 0; i < 3; i++) {

    if (timerEditField == i) {

      oled.drawBox(
        0,
        y[i] - 11,
        128,
        14
      );

      oled.setDrawColor(0);
    }

    // --------------------------------------------------------
    // При редактировании времени мигаем только
    // часами / минутами
    // --------------------------------------------------------

    if (
      i < 2 &&
      timerValueEditing &&
      timerEditField == i
    ) {

      int value =
        (i == 0)
          ? outputTimers[timerPos].onMinutes
          : outputTimers[timerPos].offMinutes;

      int hours =
        value / 60;

      int minutes =
        value % 60;

      const char* prefix =
        (i == 0)
          ? "ВКЛ:  "
          : "ВЫКЛ: ";

      oled.drawUTF8(
        2,
        y[i],
        prefix
      );

      char part[4];

      snprintf(
        part,
        sizeof(part),
        "%02d",
        hours
      );

      if (
        timerSubField != 0 ||
        timerBlinkState
      )
        oled.drawStr(
          44,
          y[i],
          part
        );

      oled.drawStr(
        58,
        y[i],
        ":"
      );

      snprintf(
        part,
        sizeof(part),
        "%02d",
        minutes
      );

      if (
        timerSubField != 1 ||
        timerBlinkState
      )
        oled.drawStr(
          65,
          y[i],
          part
        );
    }

    else {

      oled.drawUTF8(
        2,
        y[i],
        lines[i]
      );
    }

    if (timerEditField == i)
      oled.setDrawColor(1);
  }
}

// ============================================================
// DRAW CLOCK
// ============================================================

void drawClock() {

  DateTime now =
    rtc.now();

  oled.setFont(
    u8g2_font_unifont_t_cyrillic
  );

  oled.drawUTF8(
    0,
    13,
    "ЧАСЫ"
  );

  oled.setFont(
    u8g2_font_10x20_t_cyrillic
  );

  char timeText[10];

  snprintf(
    timeText,
    sizeof(timeText),
    "%02d:%02d:%02d",
    now.hour(),
    now.minute(),
    now.second()
  );

  oled.drawStr(
    17,
    39,
    timeText
  );

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  char dateText[16];

  snprintf(
    dateText,
    sizeof(dateText),
    "%02d.%02d.%04d",
    now.day(),
    now.month(),
    now.year()
  );

  oled.drawStr(
    32,
    58,
    dateText
  );
}

// ============================================================
// DRAW WIFI
// ============================================================

void drawWiFi() {

  oled.setFont(
    u8g2_font_unifont_t_cyrillic
  );

  oled.drawUTF8(
    0,
    13,
    "WI-FI"
  );

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  if (
    WiFi.status() ==
    WL_CONNECTED
  ) {

    char signalText[20];

    snprintf(
      signalText,
      sizeof(signalText),
      "%d dBm",
      WiFi.RSSI()
    );

    oled.drawStr(
      62,
      12,
      signalText
    );

    oled.drawUTF8(
      0,
      28,
      "Подключён"
    );

    oled.drawUTF8(
      0,
      43,
      savedSSID.c_str()
    );

    oled.drawUTF8(
      0,
      58,
      "PUSH: сменить сеть"
    );
  }

  else {

    oled.drawUTF8(
      0,
      28,
      "Нет подключения"
    );

    if (
      savedSSID.length() > 0
    ) {

      oled.drawUTF8(
        0,
        43,
        "Сохранена:"
      );

      oled.drawUTF8(
        0,
        58,
        savedSSID.c_str()
      );
    }

    else {

      oled.drawUTF8(
        0,
        43,
        "Сеть не сохранена"
      );

      oled.drawUTF8(
        0,
        58,
        "PUSH: поиск сети"
      );
    }
  }
}

// ============================================================
// DRAW WIFI SCAN
// ============================================================

void drawWiFiScan() {

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  oled.drawUTF8(
    0,
    11,
    "ВЫБОР СЕТИ"
  );

  if (wifiCount == 0) {

    oled.drawUTF8(
      0,
      32,
      "Сетей нет"
    );

    return;
  }

  int first =
    wifiPos - 2;

  if (first < 0)
    first = 0;

  if (
    first >
    wifiCount - 4
  )
    first =
      wifiCount - 4;

  for (
    int i = 0;
    i < 4;
    i++
  ) {

    int n =
      first + i;

    if (
      n >= wifiCount
    )
      break;

    int y =
      25 + i * 12;

    if (n == wifiPos) {

      oled.drawBox(
        0,
        y - 10,
        128,
        12
      );

      oled.setDrawColor(0);

      oled.drawUTF8(
        3,
        y,
        wifiNames[n].c_str()
      );

      oled.setDrawColor(1);
    }

    else {

      oled.drawUTF8(
        3,
        y,
        wifiNames[n].c_str()
      );
    }
  }
}

// ============================================================
// DRAW WIFI PASSWORD
// ============================================================

void drawWiFiPassword() {

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  oled.drawUTF8(
    0,
    10,
    "ПАРОЛЬ"
  );

  String ssid =
    wifiNames[wifiPos];

  if (ssid.length() > 18)
    ssid =
      ssid.substring(0, 18);

  oled.drawUTF8(
    0,
    22,
    ssid.c_str()
  );

  String visiblePassword =
    wifiPassword;

  if (
    visiblePassword.length() > 18
  ) {

    visiblePassword =
      "..." +
      visiblePassword.substring(
        visiblePassword.length() - 15
      );
  }

  oled.drawStr(
    0,
    34,
    visiblePassword.c_str()
  );

  // ----------------------------------------------------------
  // КАРУСЕЛЬ
  // ----------------------------------------------------------

  int count =
    PASSWORD_CHAR_COUNT;

  const int SIDE_COUNT = 8;

  char c[2];

  c[1] = '\0';

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  int totalWidth = 0;

  for (
    int i = -SIDE_COUNT;
    i < 0;
    i++
  ) {

    int pos =
      (
        passwordCharPos +
        i +
        count
      ) % count;

    c[0] =
      passwordChars[pos];

    totalWidth +=
      oled.getStrWidth(c);
  }

  c[0] =
    passwordChars[passwordCharPos];

  oled.setFont(
    u8g2_font_10x20_t_cyrillic
  );

  int selectedWidth =
    oled.getStrWidth(c);

  int selectedBoxWidth =
    selectedWidth + 4;

  totalWidth +=
    selectedBoxWidth;

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  for (
    int i = 1;
    i <= SIDE_COUNT;
    i++
  ) {

    int pos =
      (
        passwordCharPos +
        i
      ) % count;

    c[0] =
      passwordChars[pos];

    totalWidth +=
      oled.getStrWidth(c);
  }

  int x =
    (128 - totalWidth) / 2;

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  for (
    int i = -SIDE_COUNT;
    i < 0;
    i++
  ) {

    int pos =
      (
        passwordCharPos +
        i +
        count
      ) % count;

    c[0] =
      passwordChars[pos];

    oled.drawStr(
      x,
      48,
      c
    );

    x +=
      oled.getStrWidth(c);
  }

  c[0] =
    passwordChars[passwordCharPos];

  oled.setFont(
    u8g2_font_10x20_t_cyrillic
  );

  selectedWidth =
    oled.getStrWidth(c);

  selectedBoxWidth =
    selectedWidth + 4;

  oled.setDrawColor(1);

  oled.drawBox(
    x,
    33,
    selectedBoxWidth,
    20
  );

  oled.setDrawColor(0);

  oled.drawStr(
    x + 2,
    50,
    c
  );

  oled.setDrawColor(1);

  x +=
    selectedBoxWidth;

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  for (
    int i = 1;
    i <= SIDE_COUNT;
    i++
  ) {

    int pos =
      (
        passwordCharPos +
        i
      ) % count;

    c[0] =
      passwordChars[pos];

    oled.drawStr(
      x,
      48,
      c
    );

    x +=
      oled.getStrWidth(c);
  }

  oled.setFont(
    u8g2_font_6x12_t_cyrillic
  );

  oled.drawUTF8(
    0,
    63,
    "долгое наж. - ОК"
  );
}

// ============================================================
// DRAW WIFI CONNECTING
// ============================================================

void drawWiFiConnecting() {

  oled.setFont(
    u8g2_font_unifont_t_cyrillic
  );

  oled.drawUTF8(
    0,
    14,
    "ПОДКЛЮЧЕНИЕ"
  );

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  oled.drawUTF8(
    0,
    31,
    wifiNames[wifiPos].c_str()
  );

  oled.drawUTF8(
    0,
    48,
    "Подождите..."
  );
}

// ============================================================
// DRAW SETTINGS
// ============================================================

void drawSettings() {

  oled.setFont(
    u8g2_font_7x13_t_cyrillic
  );

  char line[40];

  // ----------------------------------------------------------
  // Заряженный
  // ----------------------------------------------------------

  snprintf(
    line,
    sizeof(line),
    "Заряженный %.2fV",
    batteryFullVoltage
  );

  if (
    settingsPos == 0
  ) {

    oled.drawBox(
      0,
      1,
      128,
      14
    );

    oled.setDrawColor(0);

    oled.drawUTF8(
      2,
      12,
      line
    );

    oled.setDrawColor(1);
  }

  else {

    oled.drawUTF8(
      2,
      12,
      line
    );
  }

  // ----------------------------------------------------------
  // Критический
  // ----------------------------------------------------------

  snprintf(
    line,
    sizeof(line),
    "Разряженный %.2fV",
    batteryCriticalVoltage
  );

  if (
    settingsPos == 1
  ) {

    oled.drawBox(
      0,
      16,
      128,
      14
    );

    oled.setDrawColor(0);

    oled.drawUTF8(
      2,
      27,
      line
    );

    oled.setDrawColor(1);
  }

  else {

    oled.drawUTF8(
      2,
      27,
      line
    );
  }

  // ----------------------------------------------------------
  // Коррекция
  // ----------------------------------------------------------

  snprintf(
    line,
    sizeof(line),
    "Кор.напряж. %+.2fV",
    batteryCorrection
  );

  if (
    settingsPos == 2
  ) {

    oled.drawBox(
      0,
      31,
      128,
      14
    );

    oled.setDrawColor(0);

    oled.drawUTF8(
      2,
      42,
      line
    );

    oled.setDrawColor(1);
  }

  else {

    oled.drawUTF8(
      2,
      42,
      line
    );
  }

  // ----------------------------------------------------------
  // Задержка
  // ----------------------------------------------------------

  snprintf(
    line,
    sizeof(line),
    "Задержка %dмин",
    batteryDelayMinutes
  );

  if (
    settingsPos == 3
  ) {

    oled.drawBox(
      0,
      46,
      128,
      14
    );

    oled.setDrawColor(0);

    oled.drawUTF8(
      2,
      57,
      line
    );

    oled.setDrawColor(1);
  }

  else {

    oled.drawUTF8(
      2,
      57,
      line
    );
  }

  // ----------------------------------------------------------
  // Рамка редактирования
  // ----------------------------------------------------------

  if (settingsEditing) {

    oled.setDrawColor(1);

    oled.drawFrame(
      0,
      0,
      128,
      62
    );
  }
}

// ============================================================
// DRAW SCREEN
// ============================================================

void drawScreen() {

  oled.clearBuffer();

  switch (screen) {

    case SCREEN_MAIN:
      drawMain();
      break;

    case SCREEN_MENU:
      drawMenu();
      break;

    case SCREEN_BATTERY:
      drawBattery();
      break;

    case SCREEN_OUTPUTS:
      drawOutputs();
      break;

    case SCREEN_SCHEDULE:
      drawSchedule();
      break;

    case SCREEN_CLOCK:
      drawClock();
      break;

    case SCREEN_WIFI:
      drawWiFi();
      break;

    case SCREEN_WIFI_SCAN:
      drawWiFiScan();
      break;

    case SCREEN_WIFI_PASSWORD:
      drawWiFiPassword();
      break;

    case SCREEN_WIFI_CONNECTING:
      drawWiFiConnecting();
      break;

    case SCREEN_SETTINGS:
      drawSettings();
      break;
  }

  oled.sendBuffer();
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(300);

  Serial.println();
  Serial.println("================================");
  Serial.println("DUNDEE CONTROLLER");
  Serial.println("================================");

  // ----------------------------------------------------------
  // GPIO
  // ----------------------------------------------------------

  pinMode(
    ENC_A,
    INPUT
  );

  pinMode(
    ENC_B,
    INPUT
  );

  pinMode(
    ENC_PUSH,
    INPUT_PULLUP
  );

  pinMode(
    BTN_BACK,
    INPUT_PULLUP
  );

  // ----------------------------------------------------------
  // ВЫХОДЫ
  //
  // D8 = Нагрузка
  // D4 = Зарядка
  // D3 = Резерв 1
  // ----------------------------------------------------------

  pinMode(
    MOSFET1_PIN,
    OUTPUT
  );

  pinMode(
    MOSFET2_PIN,
    OUTPUT
  );

  pinMode(
    MOSFET3_PIN,
    OUTPUT
  );

  digitalWrite(
    MOSFET1_PIN,
    LOW
  );

  digitalWrite(
    MOSFET2_PIN,
    LOW
  );

  digitalWrite(
    MOSFET3_PIN,
    LOW
  );

  // Начальное состояние зарядки для детектора
  // момента её включения.
  previousChargeState = false;

  lastState =
    (digitalRead(ENC_A) << 1) |
     digitalRead(ENC_B);

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    D2,
    D1
  );

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  oled.begin();

  // ----------------------------------------------------------
  // RTC
  // ----------------------------------------------------------

  if (!rtc.begin()) {

    Serial.println(
      "RTC ERROR!"
    );
  }

  else {

    Serial.println(
      "RTC OK"
    );

    if (rtc.lostPower()) {

      Serial.println(
        "RTC lost power."
      );

      Serial.println(
        "Setting compile time..."
      );

      rtc.adjust(
        DateTime(
          F(__DATE__),
          F(__TIME__)
        )
      );
    }
  }

  // ----------------------------------------------------------
  // LittleFS
  // ----------------------------------------------------------

  if (!LittleFS.begin()) {

    Serial.println(
      "LittleFS ERROR!"
    );
  }

  else {

    Serial.println(
      "LittleFS OK"
    );

    loadWiFiCredentials();

    loadBatterySettings();

    loadTimerSettings();
  }

  // ----------------------------------------------------------
  // Первый замер батареи
  // ----------------------------------------------------------

  updateBattery();

  // ----------------------------------------------------------
  // WIFI AUTO CONNECT
  // ----------------------------------------------------------

  if (
    savedSSID.length() > 0
  ) {

    if (
      connectSavedWiFi()
    ) {

      syncRTCFromNTP();

      Serial.println(
        "RTC continues autonomously."
      );
    }
  }

  // ----------------------------------------------------------
  // MAIN SCREEN
  // ----------------------------------------------------------

  drawScreen();

  Serial.println();
  Serial.println("================================");
  Serial.println("READY");
  Serial.println("================================");
}

// ============================================================
// OUTPUT TIMERS
// ============================================================

void updateOutputTimers() {

  DateTime now = rtc.now();

  int currentMinute =
    now.hour() * 60 + now.minute();

  int currentDay =
    now.day();

  if (
    currentMinute == lastTimerMinute &&
    currentDay == lastTimerDay
  )
    return;

  lastTimerMinute = currentMinute;
  lastTimerDay = currentDay;

  // ==========================================================
  // LOAD TIMER
  // ==========================================================

  if (outputTimers[0].enabled) {

    if (currentMinute == outputTimers[0].onMinutes) {

      if (digitalRead(MOSFET1_PIN)) {

        Serial.println(
          "TIMER LOAD ON: ALREADY ON"
        );
      }

      else if (
        batteryVoltage <=
        batteryCriticalVoltage
      ) {

        Serial.println();
        Serial.println(
          "TIMER LOAD ON BLOCKED"
        );

        Serial.print(
          "Voltage: "
        );

        Serial.print(
          batteryVoltage,
          2
        );

        Serial.println(
          " V"
        );

        Serial.println(
          "Reason: BATTERY CRITICAL"
        );
      }

      else {

        digitalWrite(
          MOSFET1_PIN,
          HIGH
        );

        Serial.println();
        Serial.println(
          "TIMER: LOAD -> BATTERY"
        );

        Serial.print(
          "Time: "
        );

        Serial.printf(
          "%02d:%02d\n",
          now.hour(),
          now.minute()
        );
      }
    }

    if (currentMinute == outputTimers[0].offMinutes) {

      if (!digitalRead(MOSFET1_PIN)) {

        Serial.println(
          "TIMER LOAD OFF: ALREADY OFF"
        );
      }

      else {

        digitalWrite(
          MOSFET1_PIN,
          LOW
        );

        Serial.println();
        Serial.println(
          "TIMER: LOAD -> GRID"
        );

        Serial.print(
          "Time: "
        );

        Serial.printf(
          "%02d:%02d\n",
          now.hour(),
          now.minute()
        );
      }
    }
  }

  // ==========================================================
  // CHARGER TIMER
  // ==========================================================

  if (outputTimers[1].enabled) {

    if (currentMinute == outputTimers[1].onMinutes) {

      if (digitalRead(MOSFET2_PIN)) {

        Serial.println(
          "TIMER CHARGE ON: ALREADY ON"
        );
      }

      else {

        digitalWrite(
          MOSFET2_PIN,
          HIGH
        );

        Serial.println();
        Serial.println(
          "TIMER: CHARGER -> ON"
        );

        Serial.print(
          "Time: "
        );

        Serial.printf(
          "%02d:%02d\n",
          now.hour(),
          now.minute()
        );
      }
    }

    if (currentMinute == outputTimers[1].offMinutes) {

      if (!digitalRead(MOSFET2_PIN)) {

        Serial.println(
          "TIMER CHARGE OFF: ALREADY OFF"
        );
      }

      else {

        digitalWrite(
          MOSFET2_PIN,
          LOW
        );

        Serial.println();
        Serial.println(
          "TIMER: CHARGER -> OFF"
        );

        Serial.print(
          "Time: "
        );

        Serial.printf(
          "%02d:%02d\n",
          now.hour(),
          now.minute()
        );
      }
    }
  }

  // ==========================================================
  // RESERVE 1 TIMER
  // ==========================================================

  if (outputTimers[2].enabled) {

    if (currentMinute == outputTimers[2].onMinutes) {

      if (digitalRead(MOSFET3_PIN)) {

        Serial.println(
          "TIMER RESERVE1 ON: ALREADY ON"
        );
      }

      else {

        digitalWrite(
          MOSFET3_PIN,
          HIGH
        );

        Serial.println();
        Serial.println(
          "TIMER: RESERVE 1 -> ON"
        );

        Serial.print(
          "Time: "
        );

        Serial.printf(
          "%02d:%02d\n",
          now.hour(),
          now.minute()
        );
      }
    }

    if (currentMinute == outputTimers[2].offMinutes) {

      if (!digitalRead(MOSFET3_PIN)) {

        Serial.println(
          "TIMER RESERVE1 OFF: ALREADY OFF"
        );
      }

      else {

        digitalWrite(
          MOSFET3_PIN,
          LOW
        );

        Serial.println();
        Serial.println(
          "TIMER: RESERVE 1 -> OFF"
        );

        Serial.print(
          "Time: "
        );

        Serial.printf(
          "%02d:%02d\n",
          now.hour(),
          now.minute()
        );
      }
    }
  }

  // ----------------------------------------------------------
  // Резерв 2 — пока не используется.
  // ----------------------------------------------------------
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Управление
  // ----------------------------------------------------------

  updateEncoder();

  updatePush();

  updateBack();

  // ----------------------------------------------------------
  // 5 секунд бездействия —
  // убрать выбор выхода
  // ----------------------------------------------------------

  if (
    screen == SCREEN_MAIN &&
    mainOutputSelectMode &&
    millis() - lastMainOutputActivity >=
      MAIN_OUTPUT_TIMEOUT
  ) {

    mainOutputSelectMode = false;
    mainOutputSelection = 0;
    drawScreen();
  }

  // ----------------------------------------------------------
  // Батарея
  // ----------------------------------------------------------

  updateBattery();

  updateOutputTimers();

  // ----------------------------------------------------------
  // Мигание часов / минут при редактировании таймера
  // ----------------------------------------------------------

  if (
    screen == SCREEN_SCHEDULE &&
    timerEditing &&
    timerValueEditing &&
    timerEditField < 2
  ) {

    if (
      millis() - timerBlinkMillis >= 400
    ) {

      timerBlinkMillis = millis();
      timerBlinkState = !timerBlinkState;
    }
  }

  else {

    timerBlinkState = true;
  }

  // ----------------------------------------------------------
  // Динамическое обновление экрана
  // ----------------------------------------------------------

  static unsigned long lastDraw = 0;

  if (
    millis() - lastDraw >=
    250
  ) {

    lastDraw = millis();

    drawScreen();
  }

  delay(2);
}