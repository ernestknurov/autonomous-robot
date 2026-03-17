#include <Arduino.h>
#include "config.h"
#include "wifi.h"
#include "motors.h"
#include "utils.h"

// Forward declaration
void wifi_println(const String& msg, int clientIndex);

void bootBlink() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void setup() {
  // LED init
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  bootBlink();
  ultrasonic_init();
  wifi_init();
  motors_init();
  
  Serial.println("ESP32 Robot Ready!");
}

void loop() {
  ultrasonic_update();  // Check distance every 20ms
  motors_update();      // Handle non-blocking motor timing
  
  // Check if motors just completed and broadcast DONE
  if (motors_just_completed()) {
    wifi_println("DONE", -1);  // -1 broadcasts to all clients
  }
  
  wifi_poll();           // Handle WiFi client connections and commands
}

