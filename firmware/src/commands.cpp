#include <Arduino.h>
#include "commands.h"
#include "motors.h"
#include "utils.h"

// Forward declaration
void wifi_println(const String& msg, int clientIndex);

// ----------------- small helpers -----------------

static bool is_whitespace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

// Parse a signed float from Arduino String.
// Arduino's String::toFloat() returns 0.0 for both "0" and invalid input,
// so we validate by checking that the string contains at least one digit.
static bool parse_signed_float(const String& s, float& out) {
  String t = s;
  t.trim();
  if (t.length() == 0) return false;

  bool hasDigit = false;
  for (int i = 0; i < t.length(); i++) {
    char c = t[i];
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      break;
    }
  }
  if (!hasDigit) return false;

  out = t.toFloat();
  return true;
}

// Parse a signed integer (int16 range).
// Similar validation: must contain at least one digit.
static bool parse_signed_int16(const String& s, int16_t& out) {
  String t = s;
  t.trim();
  if (t.length() == 0) return false;

  bool hasDigit = false;
  for (int i = 0; i < t.length(); i++) {
    char c = t[i];
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      break;
    }
  }
  if (!hasDigit) return false;

  long v = t.toInt(); // handles leading '+' or '-'
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;

  out = (int16_t)v;
  return true;
}

// Split command into: keyword + argument string (rest of line).
// Example: "MOVE 1.23" => keyword="MOVE", arg="1.23"
static void split_keyword_arg(const String& line, String& keyword, String& arg) {
  String t = line;
  t.trim();

  int n = t.length();
  int i = 0;

  // find end of keyword
  while (i < n && !is_whitespace(t[i])) i++;

  keyword = t.substring(0, i);
  keyword.trim();
  keyword.toUpperCase();

  // skip spaces
  while (i < n && is_whitespace(t[i])) i++;

  arg = (i < n) ? t.substring(i) : "";
  arg.trim();
}

// ----------------- public API -----------------

Command parse_command(const String& raw) {
  Command c;
  c.raw = raw;

  String line = raw;
  line.trim();
  if (line.length() == 0) {
    c.type = CommandType::UNKNOWN;
    return c;
  }

  String keyword, arg;
  split_keyword_arg(line, keyword, arg);

  if (keyword == "LED") {
    // support: "LED ON" / "LED OFF"
    String a = arg;
    a.trim();
    a.toUpperCase();
    if (a == "ON") {
      c.type = CommandType::LED_ON;
      return c;
    }
    if (a == "OFF") {
      c.type = CommandType::LED_OFF;
      return c;
    }
    c.type = CommandType::UNKNOWN;
    return c;
  }
  
  if (keyword == "GET_DISTANCE") {
    c.type = CommandType::GET_DISTANCE;
    return c;
  }

  if (keyword == "PLAY_SOUND") {
    c.type = CommandType::PLAY_SOUND;
    return c;
  }

  if (keyword == "MOVE") {
    float meters = 0.0f;
    if (!parse_signed_float(arg, meters)) {
      c.type = CommandType::UNKNOWN;
      return c;
    }
    c.type = CommandType::MOVE;
    c.meters = meters; // signed
    return c;
  }

  if (keyword == "TURN") {
    int16_t deg = 0;
    if (!parse_signed_int16(arg, deg)) {
      c.type = CommandType::UNKNOWN;
      return c;
    }
    c.type = CommandType::TURN;
    c.deg = deg; // signed
    return c;
  }

  c.type = CommandType::UNKNOWN;
  return c;
}

// Helper to print to Serial and WiFi client
static void printToClient(const String& msg, int clientIndex) {
  wifi_println(msg, clientIndex);
}

void execute_command(const Command& c, int clientIndex) {
  // Prevent overlapping motor commands
  if ((c.type == CommandType::MOVE || c.type == CommandType::TURN) && motors_busy()) {
    printToClient("ERR:BUSY", clientIndex);
    return;
  }

  switch (c.type) {
    case CommandType::LED_ON:
      digitalWrite(LED_BUILTIN, HIGH);
      printToClient("OK: LED ON", clientIndex);
      printToClient("DONE", clientIndex);
      break;

    case CommandType::LED_OFF:
      digitalWrite(LED_BUILTIN, LOW);
      printToClient("OK: LED OFF", clientIndex);
      printToClient("DONE", clientIndex);
      break;
    
    case CommandType::GET_DISTANCE: {
      float dist = getDistance();
      String msg = "OK: DISTANCE " + String(dist, 3);
      printToClient(msg, clientIndex);
      printToClient("DONE", clientIndex);
      break;
    }

    case CommandType::PLAY_SOUND:
      playMissionCompleteSound();
      printToClient("OK: PLAY SOUND", clientIndex);
      printToClient("DONE", clientIndex);
      break;

    case CommandType::MOVE: {
      float m = c.meters;
      String msg = "OK: MOVE " + String(m, 3);
      printToClient(msg, clientIndex);

      if (m > 0.0f) {
        moveForward(m);
        String distMsg = "Distance to obstacle: " + String(getDistance(), 3);
        printToClient(distMsg, clientIndex);
      } else if (m < 0.0f) {
        moveBackward(-m);
      } else {
        // MOVE 0: do nothing, send DONE immediately
        printToClient("DONE", clientIndex);
      }
      // DONE will be sent by main loop after motors complete
      break;
    }

    case CommandType::TURN: {
      int16_t d = c.deg;
      String msg = "OK: TURN " + String((int)d);
      printToClient(msg, clientIndex);

      if (d > 0) {
        // "default right"
        uint8_t a = (d > 255) ? 255 : (uint8_t)d;
        turnRight(a);
      } else if (d < 0) {
        // negative => left
        int16_t ad = (int16_t)(-d);
        uint8_t a = (ad > 255) ? 255 : (uint8_t)ad;
        turnLeft(a);
      } else {
        // TURN 0: do nothing, send DONE immediately
        printToClient("DONE", clientIndex);
      }
      // DONE will be sent by main loop after motors complete
      break;
    }

    default:
      String errMsg = "ERR:UNKNOWN " + c.raw;
      printToClient(errMsg, clientIndex);
      break;
  }
}

void handle_command(const String& raw, int clientIndex) {
  Command c = parse_command(raw);
  execute_command(c, clientIndex);
}
