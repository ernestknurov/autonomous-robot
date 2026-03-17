#pragma once

// motion
constexpr float SPEED_MPS = 2.5f; // from calibration, m/s
constexpr float ANGULAR_SPEED_DPS = 214.0f; // calibrate later (deg/s)

// timing
constexpr int BRAKE_TIME_MS = 120;

// obstacle detection
constexpr unsigned long DISTANCE_CHECK_INTERVAL_MS = 20; // check distance every 60ms
constexpr float OBSTACLE_THRESHOLD_CM = 20.0f; // stop if closer than 20cm

// ESP32 pins (adapted from Arduino Nano)
constexpr int ECHO = 35;  // ultrasonic sensor echo
constexpr int TRIG = 32;   // ultrasonic sensor trigger
constexpr int IN1 = 33;   // left motor forward
constexpr int IN2 = 25;   // left motor backward
constexpr int IN3 = 26;   // right motor forward
constexpr int IN4 = 27;   // right motor backward

// LED (ESP32 built-in LED is typically on GPIO 2)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// WiFi Configuration
constexpr const char* WIFI_SSID = "HUAWEI-A1-E5E047-2.4";           // Change this to your WiFi network name
constexpr const char* WIFI_PASSWORD = "53404656";   // Change this to your WiFi password
constexpr int TCP_PORT = 8080;                              // TCP server port
constexpr int MAX_CLIENTS = 1;                               // Maximum simultaneous connections
