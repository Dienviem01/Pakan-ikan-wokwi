#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ===================== Konfigurasi Hardware =====================
#define PIN_OLED_SDA      14
#define PIN_OLED_SCL      13
#define OLED_ADDR         0x3C
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64

#define PIN_POT           16   // Potentiometer (ADC)
#define PIN_SERVO         17   // Servo PWM (sesuai wiring)
#define PIN_LED           2    // LED indikator feeding
#define PIN_BUZZER        1    // Buzzer (stok hampir habis)
#define PIN_BUTTON_FEED   21   // Push button feeding (ke GND, INPUT_PULLUP)
#define PIN_BUTTON_REFILL 20   // Push button refill tank (ke GND, INPUT_PULLUP)

// Servo angle (sesuaikan kalau perlu)
#define SERVO_CLOSED_ANGLE  0
#define SERVO_OPEN_ANGLE    90

// Interval feeding otomatis (ms) - tiap 10 detik untuk simulasi
const unsigned long AUTO_FEED_INTERVAL_MS = 10000; // 10 detik

// ===================== Objek Global =====================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo feederServo;

// Mutex & Semaphore FreeRTOS
SemaphoreHandle_t mutexState;        // Lindungi shared state
SemaphoreHandle_t semFeedRequest;    // Event permintaan feeding (manual/auto)

// Waktu terakhir feeding otomatis dijadwalkan
unsigned long lastAutoFeedMs = 0;

// Flag global untuk buzzer (low stock)
volatile bool g_lowStock = false;

// ===================== Shared State =====================
struct SystemState {
  float   tankLevelPercent;  // 0–100
  uint8_t feedPercent;       // 1–20, di-set dari potentiometer
  bool    isFeeding;         // true saat feeding berlangsung
  uint32_t msToNextFeed;     // ms menuju feeding otomatis berikutnya
};

SystemState state;

// ===================== Deklarasi Task =====================
void TaskInput(void *pvParameters);
void TaskFeeder(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskBuzzer(void *pvParameters);

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Fish Feeder ESP32-S3 - Multicore + Semaphore + Mutex");

  // Pin mode
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON_FEED, INPUT_PULLUP);   // button ke GND → aktif LOW
  pinMode(PIN_BUTTON_REFILL, INPUT_PULLUP); // button ke GND → aktif LOW

  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // I2C + OLED
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed!");
    while (true) {
      delay(100);
    }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Fish Feeder");
  display.println("Init...");
  display.display();

  // Servo
  feederServo.attach(PIN_SERVO);
  feederServo.write(SERVO_CLOSED_ANGLE);

  // Inisialisasi state awal
  state.tankLevelPercent = 100.0f;               // kapasitas awal 100%
  state.feedPercent      = 10;                   // default 10%
  state.isFeeding        = false;
  state.msToNextFeed     = AUTO_FEED_INTERVAL_MS;

  lastAutoFeedMs = millis();

  // Buat mutex & semaphore
  mutexState = xSemaphoreCreateMutex();
  semFeedRequest = xSemaphoreCreateBinary();

  if (mutexState == NULL || semFeedRequest == NULL) {
    Serial.println("Gagal membuat mutex / semaphore!");
    while (true) {
      delay(100);
    }
  }

  // ===================== Buat Task di Multicore =====================
  // TaskInput  -> Core 0
  xTaskCreatePinnedToCore(
    TaskInput,
    "TaskInput",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  // TaskDisplay -> Core 0
  xTaskCreatePinnedToCore(
    TaskDisplay,
    "TaskDisplay",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  // TaskBuzzer -> Core 0 (ringan, cukup core yang sama)
  xTaskCreatePinnedToCore(
    TaskBuzzer,
    "TaskBuzzer",
    2048,
    NULL,
    1,
    NULL,
    0
  );

  // TaskFeeder -> Core 1 (lebih tinggi prioritas karena menggerakkan servo)
  xTaskCreatePinnedToCore(
    TaskFeeder,
    "TaskFeeder",
    4096,
    NULL,
    2,
    NULL,
    1
  );
}

void loop() {
  // Tidak dipakai karena kita pakai FreeRTOS tasks.
  vTaskDelay(portMAX_DELAY);
}

// ===================== Task: Input (Pot + Button + Jadwal + Refill) =====================
void TaskInput(void *pvParameters) {
  (void) pvParameters;

  bool lastFeedButtonPressed   = false;
  bool lastRefillButtonPressed = false;

  for (;;) {
    unsigned long now = millis();

    // ---- Baca potentiometer -> feedPercent (1–20%) ----
    int raw = analogRead(PIN_POT); // 0–4095
    int feedPercent = map(raw, 0, 4095, 1, 20);
    if (feedPercent < 1)  feedPercent = 1;
    if (feedPercent > 20) feedPercent = 20;

    // ---- Button feeding manual ----
    bool feedButtonPressed = (digitalRead(PIN_BUTTON_FEED) == LOW); // aktif LOW

    if (feedButtonPressed && !lastFeedButtonPressed) {
      Serial.println("Manual feeding requested (button)");
      xSemaphoreGive(semFeedRequest);
    }
    lastFeedButtonPressed = feedButtonPressed;

    // ---- Feeding otomatis berdasarkan interval ----
    uint32_t msToNextFeed;
    unsigned long elapsed = now - lastAutoFeedMs;

    if (elapsed >= AUTO_FEED_INTERVAL_MS) {
      Serial.println("Auto feeding requested (scheduler)");
      xSemaphoreGive(semFeedRequest);
      lastAutoFeedMs = now;
      msToNextFeed = AUTO_FEED_INTERVAL_MS;
    } else {
      msToNextFeed = AUTO_FEED_INTERVAL_MS - elapsed;
    }

    // ---- Update feedPercent & msToNextFeed ke shared state ----
    if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(50)) == pdTRUE) {
      state.feedPercent   = (uint8_t)feedPercent;
      state.msToNextFeed  = msToNextFeed;
      xSemaphoreGive(mutexState);
    }

    // ---- Button refill tank (set tank 100%) ----
    bool refillPressed = (digitalRead(PIN_BUTTON_REFILL) == LOW); // aktif LOW
    if (refillPressed && !lastRefillButtonPressed) {
      Serial.println("Refill tank requested (button)");
      if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(100)) == pdTRUE) {
        state.tankLevelPercent = 100.0f;
        xSemaphoreGive(mutexState);
      }
    }
    lastRefillButtonPressed = refillPressed;

    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms
  }
}

// ===================== Task: Feeder (Servo + LED + Update Tank) =====================
void TaskFeeder(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    if (xSemaphoreTake(semFeedRequest, portMAX_DELAY) == pdTRUE) {
      uint8_t localFeedPercent = 0;
      float   localTankLevel   = 0.0f;

      // Ambil data yang diperlukan dari shared state
      if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(100)) == pdTRUE) {
        localFeedPercent = state.feedPercent;
        localTankLevel   = state.tankLevelPercent;
        state.isFeeding  = true;   // status feeding ON
        xSemaphoreGive(mutexState);
      }

      Serial.print("Feeding ");
      Serial.print(localFeedPercent);
      Serial.println("% ... (request accepted)");

      // Nyalakan LED indikator
      digitalWrite(PIN_LED, HIGH);

      // Gerakkan servo: open -> hold -> close
      int openTimeMs = 300 + localFeedPercent * 70;

      feederServo.write(SERVO_OPEN_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(openTimeMs));
      feederServo.write(SERVO_CLOSED_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(300));

      // Hitung tank level baru (clamp minimal 0)
      float newTankLevel = localTankLevel - (float)localFeedPercent;
      if (newTankLevel < 0.0f) newTankLevel = 0.0f;

      if (newTankLevel == 0.0f) {
        Serial.println("Tank is now EMPTY after this feeding.");
      }

      // Tulis balik ke shared state
      if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(100)) == pdTRUE) {
        state.tankLevelPercent = newTankLevel;
        state.isFeeding        = false;   // selesai feeding
        xSemaphoreGive(mutexState);
      }

      // Matikan LED
      digitalWrite(PIN_LED, LOW);

      Serial.print("Feeding done. Tank level: ");
      Serial.print(newTankLevel);
      Serial.println("%");
    }
  }
}

// ===================== Task: Display (OLED) =====================
void TaskDisplay(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    float   tankLevel      = 0.0f;
    uint8_t feedPercent    = 0;
    bool    isFeeding      = false;
    uint32_t msToNextFeed  = 0;

    if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(50)) == pdTRUE) {
      tankLevel      = state.tankLevelPercent;
      feedPercent    = state.feedPercent;
      isFeeding      = state.isFeeding;
      msToNextFeed   = state.msToNextFeed;
      xSemaphoreGive(mutexState);
    }

    // Set flag global untuk buzzer (dipakai TaskBuzzer)
    g_lowStock = (tankLevel < 15.0f);

    uint16_t secToNextFeed = msToNextFeed / 1000;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);

    display.println("Fish Feeder v1.4");
    display.println("----------------");

    display.print("Tank : ");
    display.print(tankLevel, 1);
    display.println(" %");

    display.print("Feed : ");
    display.print(feedPercent);
    display.println(" %/feed");

    display.print("Next : ");
    display.print(secToNextFeed);
    display.println(" s");

    display.print("Status: ");
    if (isFeeding) {
      display.println("FEEDING");
    } else {
      display.println("IDLE");
    }

    if (tankLevel < 15.0f) {
      display.setCursor(0, 48);
      display.setTextSize(1);
      display.println("!!! LOW STOCK !!!");
    }

    display.display();

    vTaskDelay(pdMS_TO_TICKS(500)); // refresh tiap 500ms
  }
}

// ===================== Task: Buzzer (bunyi saat low stock) =====================
void TaskBuzzer(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    if (g_lowStock) {
      // Toggle cepat ~500 Hz: 1ms HIGH + 1ms LOW
      digitalWrite(PIN_BUZZER, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1));
      digitalWrite(PIN_BUZZER, LOW);
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      // Pastikan buzzer mati, cek lagi nanti
      digitalWrite(PIN_BUZZER, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}
