#include <stdint.h>
#include <Arduino.h>
#include "utils.h"

// Ultrasonic sensor state
static unsigned long lastDistanceCheck = 0;
static float lastDistance = 999.0f; // large default value

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