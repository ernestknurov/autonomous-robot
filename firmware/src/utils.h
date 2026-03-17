#pragma once
#include <stdint.h>
#include "config.h"

uint32_t metersToMillis(float distanceMeters, float speedMetersPerSec = SPEED_MPS);
int angleToMillis(uint8_t angleDeg, float speedDegPerSec = ANGULAR_SPEED_DPS);

// Ultrasonic sensor
void ultrasonic_init();
void ultrasonic_update();  // Call every loop to check distance periodically
float getDistance();       // Get distance in cm
bool isObstacleDetected(); // Check if obstacle is too close