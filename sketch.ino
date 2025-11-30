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
#define PIN_SERVO         17   // Servo PWM (sesuai wiring kamu)
#define PIN_LED           2    // LED indikator feeding
#define PIN_BUZZER        1    // Buzzer (stok hampir habis)
#define PIN_BUTTON        21   // Push button (ke GND, pakai INPUT_PULLUP)

// Servo angle (sesuaikan kalau perlu)
#define SERVO_CLOSED_ANGLE  0
#define SERVO_OPEN_ANGLE    90

// Interval feeding otomatis (ms) - SEKARANG tiap 10 detik
const unsigned long AUTO_FEED_INTERVAL_MS = 10000; // 10 detik

// ===================== Objek Global =====================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo feederServo;

// Mutex & Semaphore FreeRTOS
SemaphoreHandle_t mutexState;        // Lindungi shared state
SemaphoreHandle_t semFeedRequest;    // Event permintaan feeding (manual/auto)

// ===================== Shared State =====================
struct SystemState {
  float   tankLevelPercent;  // 0–100
  uint8_t feedPercent;       // 1–20, di-set dari potentiometer
  bool    isFeeding;         // true saat feeding berlangsung
};

SystemState state;

// ===================== Deklarasi Task =====================
void TaskInput(void *pvParameters);
void TaskFeeder(void *pvParameters);
void TaskDisplay(void *pvParameters);

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Fish Feeder ESP32-S3 - Multicore + Semaphore + Mutex");

  // Pin mode
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP); // button ke GND → aktif LOW

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
  state.tankLevelPercent = 100.0f; // kapasitas awal 100%
  state.feedPercent      = 10;     // default 10%
  state.isFeeding        = false;

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

  // setup() selesai, selanjutnya semua logika di-handle oleh task
}

void loop() {
  // Tidak dipakai karena kita pakai FreeRTOS tasks.
  vTaskDelay(portMAX_DELAY);
}

// ===================== Task: Input (Pot + Button + Jadwal) =====================
void TaskInput(void *pvParameters) {
  (void) pvParameters;

  unsigned long lastAutoFeedMs = millis();
  bool lastButtonPressed = false;

  for (;;) {
    // ---- Baca potentiometer -> feedPercent (1–20%) ----
    int raw = analogRead(PIN_POT); // 0–4095
    int feedPercent = map(raw, 0, 4095, 1, 20);
    if (feedPercent < 1)  feedPercent = 1;
    if (feedPercent > 20) feedPercent = 20;

    // Update ke shared state pakai mutex
    if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(50)) == pdTRUE) {
      state.feedPercent = (uint8_t)feedPercent;
      xSemaphoreGive(mutexState);
    }

    // ---- Baca button untuk feeding manual ----
    bool buttonPressed = (digitalRead(PIN_BUTTON) == LOW); // aktif LOW

    // Deteksi edge: dari tidak ditekan → ditekan
    if (buttonPressed && !lastButtonPressed) {
      Serial.println("Manual feeding requested (button)");
      xSemaphoreGive(semFeedRequest);
    }
    lastButtonPressed = buttonPressed;

    // ---- Feeding otomatis berdasarkan interval (sekarang 10 detik) ----
    unsigned long now = millis();
    if (now - lastAutoFeedMs >= AUTO_FEED_INTERVAL_MS) {
      Serial.println("Auto feeding requested (scheduler)");
      xSemaphoreGive(semFeedRequest);
      lastAutoFeedMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms
  }
}

// ===================== Task: Feeder (Servo + LED + Update Tank) =====================
void TaskFeeder(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Tunggu permintaan feeding (manual/jadwal)
    if (xSemaphoreTake(semFeedRequest, portMAX_DELAY) == pdTRUE) {
      uint8_t localFeedPercent = 0;
      float   localTankLevel   = 0.0f;

      // Ambil data yang diperlukan dari shared state (pakai mutex)
      if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(100)) == pdTRUE) {
        localFeedPercent = state.feedPercent;
        localTankLevel   = state.tankLevelPercent;
        state.isFeeding  = true;   // status feeding ON
        xSemaphoreGive(mutexState);
      }

      // Kalau tank kosong, boleh kasih warning
      if (localTankLevel <= 0.0f) {
        Serial.println("Tank empty, feeding skipped!");
        // kembalikan status isFeeding ke false
        if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(100)) == pdTRUE) {
          state.isFeeding = false;
          xSemaphoreGive(mutexState);
        }
        continue;
      }

      Serial.print("Feeding ");
      Serial.print(localFeedPercent);
      Serial.println("% ...");

      // Nyalakan LED indikator
      digitalWrite(PIN_LED, HIGH);

      // Gerakkan servo: open -> hold -> close
      // Lama buka servo disesuaikan dengan feedPercent
      int openTimeMs = 300 + localFeedPercent * 70; // contoh rumus sederhana

      feederServo.write(SERVO_OPEN_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(openTimeMs));
      feederServo.write(SERVO_CLOSED_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(300));

      // Update tank level berdasarkan feedPercent
      float newTankLevel = localTankLevel - (float)localFeedPercent;
      if (newTankLevel < 0.0f) newTankLevel = 0.0f;

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

// ===================== Task: Display (OLED + Buzzer) =====================
void TaskDisplay(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    float   tankLevel   = 0.0f;
    uint8_t feedPercent = 0;
    bool    isFeeding   = false;

    // Ambil snapshot state
    if (xSemaphoreTake(mutexState, pdMS_TO_TICKS(50)) == pdTRUE) {
      tankLevel   = state.tankLevelPercent;
      feedPercent = state.feedPercent;
      isFeeding   = state.isFeeding;
      xSemaphoreGive(mutexState);
    }

    // ---- Buzzer: bunyi kalau kapasitas < 15% ----
    if (tankLevel < 15.0f) {
      tone(PIN_BUZZER, 2000);  // 2kHz
    } else {
      noTone(PIN_BUZZER);
      digitalWrite(PIN_BUZZER, LOW);
    }

    // ---- Update OLED ----
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);

    display.println("Fish Feeder v1.0");
    display.println("----------------");

    display.print("Tank : ");
    display.print(tankLevel, 1);
    display.println(" %");

    display.print("Feed : ");
    display.print(feedPercent);
    display.println(" %/feed");

    display.print("Status: ");
    if (isFeeding) {
      display.println("FEEDING");
    } else {
      display.println("IDLE");
    }

    display.print("Mode : Auto+Manual");

    if (tankLevel < 15.0f) {
      display.setCursor(0, 48);
      display.setTextSize(1);
      display.println("!!! LOW STOCK !!!");
    }

    display.display();

    vTaskDelay(pdMS_TO_TICKS(500)); // refresh tiap 500ms
  }
}
