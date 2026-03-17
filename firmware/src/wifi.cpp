#include <Arduino.h>
#include <WiFi.h>
#include "wifi.h"
#include "commands.h"
#include "config.h"

WiFiServer server(TCP_PORT);
WiFiClient clients[MAX_CLIENTS];
static String line;

void wifi_init() {
  // USB Serial for debugging
  Serial.begin(115200);
  Serial.println("Initializing WiFi...");
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Start TCP server
    server.begin();
    Serial.print("TCP server started on port ");
    Serial.println(TCP_PORT);
  } else {
    Serial.println("\nFailed to connect to WiFi!");
  }
}

void wifi_poll() {
  // Check for new client connections
  WiFiClient newClient = server.available();
  if (newClient) {
    // Find an empty slot
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (!clients[i] || !clients[i].connected()) {
        clients[i] = newClient;
        Serial.print("New client connected on slot ");
        Serial.println(i);
        clients[i].println("READY");
        break;
      }
    }
  }
  
  // Handle data from connected clients
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      while (clients[i].available()) {
        char c = clients[i].read();
        if (c == '\n') {
          handle_command(line, i);
          line = "";
        } else if (c != '\r') {
          line += c;
        }
      }
    }
  }
  
  // Also handle USB Serial commands for debugging
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handle_command(line, -1);  // -1 indicates Serial (not a client)
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}

// Helper function to send message to a specific client or all clients
void wifi_println(const String& msg, int clientIndex) {
  Serial.println(msg);
  if (clientIndex >= 0 && clientIndex < MAX_CLIENTS) {
    if (clients[clientIndex] && clients[clientIndex].connected()) {
      clients[clientIndex].println(msg);
    }
  } else if (clientIndex == -1) {
    // Send to all connected clients
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (clients[i] && clients[i].connected()) {
        clients[i].println(msg);
      }
    }
  }
}
