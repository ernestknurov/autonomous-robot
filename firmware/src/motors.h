#pragma once
#include <stdint.h>

void motors_init();
void motors_update();  // Call every loop iteration to handle non-blocking motor timing
bool motors_busy();    // Check if motors are currently executing a command
bool motors_just_completed();  // Returns true if just completed
void moveForward(float meters);
void moveBackward(float meters);
void turnRight(uint8_t angleDeg);
void turnLeft(uint8_t angleDeg);
void stopAll();
void brakeAll();