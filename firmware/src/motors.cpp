#include <stdint.h>
#include <Arduino.h>
#include "config.h"
#include "utils.h"
#include "motors.h"

// Non-blocking motor state
static bool motorActive = false;
static uint32_t motorStartTime = 0;
static uint32_t motorDuration = 0;
static bool isMovingForward = false; // Track if currently moving forward
static bool justCompleted = false;  // Flag set when motors just finished

void motors_init() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopAll();
}

void motors_update() {
  if (!motorActive) return;
  
  // Check for obstacles when moving forward
  if (isMovingForward && isObstacleDetected()) {
    stopAll();
    motorActive = false;
    isMovingForward = false;
    justCompleted = true;
    Serial.println("STOPPED: Obstacle detected!");
    return;
  }
  
  if (millis() - motorStartTime >= motorDuration) {
    stopAll();
    motorActive = false;
    isMovingForward = false;
    justCompleted = true;
  }
}

bool motors_busy() {
  return motorActive;
}

bool motors_just_completed() {
  if (justCompleted) {
    justCompleted = false;
    return true;
  }
  return false;
}

void moveForward(float meters) {
  if (meters <= 0.0f) return; 

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  motorActive = true;
  isMovingForward = true;
  motorStartTime = millis();
  motorDuration = metersToMillis(meters);
}

void moveBackward(float meters) {
  if (meters <= 0.0f) return;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  motorActive = true;
  isMovingForward = false;
  motorStartTime = millis();
  motorDuration = metersToMillis(meters);
}

void turnRight(uint8_t angleDeg) {
  if (angleDeg == 0) return;

  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  motorActive = true;
  isMovingForward = false;
  motorStartTime = millis();
  motorDuration = angleToMillis(angleDeg);
}

void turnLeft(uint8_t angleDeg) {
  if (angleDeg == 0) return;

  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  motorActive = true;
  isMovingForward = false;
  motorStartTime = millis();
  motorDuration = angleToMillis(angleDeg);
}

void stopAll() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void brakeAll() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}