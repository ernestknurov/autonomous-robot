#pragma once
#include <Arduino.h>
#include <stdint.h>

enum class CommandType : uint8_t {
  LED_ON,
  LED_OFF,
  MOVE,   // signed float meters
  TURN,   // signed int degrees
  GET_DISTANCE,
  UNKNOWN
};

struct Command {
  CommandType type = CommandType::UNKNOWN;
  float meters = 0.0f;   // used when type == MOVE (signed)
  int16_t deg = 0;       // used when type == TURN (signed)
  String raw;            // optional: keep original for debug
};

// Parse raw line into a structured command.
// Does not execute any motor action.
Command parse_command(const String& raw);

// Execute a parsed command (side effects: motors, LEDs, prints).
// clientIndex: WiFi client slot (-1 for Serial/broadcast to all)
void execute_command(const Command& c, int clientIndex = -1);

// Convenience: parse + execute in one call.
void handle_command(const String& raw, int clientIndex = -1);

