#include <stdint.h>
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "utils.h"

struct ToneStep {
  uint16_t freq;       // 0 = pause
  uint16_t durationMs; // step duration
};

class BuzzerPlayer {
public:
  void begin(uint8_t pin) {
    pin_ = pin;
    #if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(pin_, 2000, 8);
    #else
        ledcSetup(BUZZER_LEDC_CHANNEL, 2000, 8);
        ledcAttachPin(pin_, BUZZER_LEDC_CHANNEL);
    #endif
    initialized_ = true;
    stop();
  }

  void playMissionComplete() {
    static const ToneStep melody[] = {
      {523, 120},   // C5
      {659, 120},   // E5
      {784, 120},   // G5
      {1046, 180},  // C6
      {0, 80},      // pause
      {784, 140},   // G5
      {1046, 220},  // C6
    };

    startMelody(melody, sizeof(melody) / sizeof(melody[0]));
  }

  void update() {
    if (!active_) return;

    uint32_t now = millis();
    if (now - stepStartMs_ < melody_[stepIndex_].durationMs) return;

    stepIndex_++;
    if (stepIndex_ >= melodyLength_) {
      stop();
      return;
    }

    stepStartMs_ = now;
    playCurrentStep();
  }

  void stop() {
    writeTone(0);
    active_ = false;
    melody_ = nullptr;
    melodyLength_ = 0;
    stepIndex_ = 0;
  }

  bool busy() const {
    return active_;
  }

private:
  static constexpr uint8_t BUZZER_LEDC_CHANNEL = 0;

  void startMelody(const ToneStep* melody, size_t length) {
    if (!initialized_ || melody == nullptr || length == 0) return;

    melody_ = melody;
    melodyLength_ = length;
    stepIndex_ = 0;
    stepStartMs_ = millis();
    active_ = true;
    playCurrentStep();
  }

  void playCurrentStep() {
    writeTone(melody_[stepIndex_].freq);
  }

  void writeTone(uint16_t freq) {
    #if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWriteTone(pin_, freq);
    #else
        ledcWriteTone(BUZZER_LEDC_CHANNEL, freq);
    #endif
  }

  uint8_t pin_ = 0;
  const ToneStep* melody_ = nullptr;
  size_t melodyLength_ = 0;
  size_t stepIndex_ = 0;
  uint32_t stepStartMs_ = 0;
  bool active_ = false;
  bool initialized_ = false;
};

// Ultrasonic sensor state
static unsigned long lastDistanceCheck = 0;
static float lastDistance = 999.0f; // large default value
static BuzzerPlayer buzzer;

void ultrasonic_init() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
}

void ultrasonic_update() {
  unsigned long now = millis();
  
  // Check distance every DISTANCE_CHECK_INTERVAL_MS
  if (now - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL_MS) {
    lastDistanceCheck = now;
    lastDistance = getDistance();
  }
}

float getDistance() {
  // Send 10us pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  // Read echo pulse duration (timeout after 30ms = ~5m max range)
  long duration = pulseIn(ECHO, HIGH, 30000);
  
  if (duration == 0) {
    return 999.0f; // timeout - assume no obstacle
  }
  
  // Calculate distance in cm (speed of sound = 343 m/s)
  // distance = (duration / 2) / 29.1 
  float distanceCm = (float)duration / 58.0f;
  
  return distanceCm;
}

bool isObstacleDetected() {
  return lastDistance < OBSTACLE_THRESHOLD_CM;
}

uint32_t metersToMillis(float distanceMeters, float speedMetersPerSec) {
  if (speedMetersPerSec <= 0.0f) {
    return 0; // protect from division by zero / invalid speed
  }

  if (distanceMeters < 0.0f) {
    distanceMeters = -distanceMeters; // keep API tolerant
  }

  float timeMs = (distanceMeters / speedMetersPerSec) * 1000.0f;
  return (uint32_t)(timeMs + 0.5f); // round to nearest (for positive values)
}

int angleToMillis(uint8_t angleDeg, float speedDegPerSec) {
  if (speedDegPerSec <= 0.0f) {
    return 0;
  }

  float timeMs = (static_cast<float>(angleDeg) / speedDegPerSec) * 1000.0f;
  return static_cast<int>(timeMs + 0.5f); // round to nearest (positive)
}

void buzzer_init() {
  buzzer.begin(BUZZER);
}

void buzzer_update() {
  buzzer.update();
}

void playMissionCompleteSound() {
  buzzer.playMissionComplete();
}

bool buzzer_busy() {
  return buzzer.busy();
}
