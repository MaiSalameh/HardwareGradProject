#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "MAISALAMEH 1760";
const char* password = "11L2:7m8";

WebServer server(80);

// Pins for communication with Arduino
#define ARDUINO_RX 19  
#define ARDUINO_TX 18

void setup() {
  Serial.begin(115200);  // Debugging
  Serial1.begin(115200, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX); // Communication with Arduino

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(1000);
    Serial.print(".");
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    while (true);  // Stop execution
  }
  
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/button", HTTP_GET, handleButtonPress);

  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();  // Handle incoming requests
}

// Handle the root route
void handleRoot() {
  server.send(200, "text/plain", "ESP32 Web Server is running!");
}

// Handle button press route
void handleButtonPress() {
  if (server.hasArg("button")) {
    String button = server.arg("button");
    Serial.println("Button clicked: " + button);

    // Send command to Arduino based on the button clicked
    Serial1.println(button);  // Send button data to Arduino

    server.send(200, "text/plain", "Button " + button + " received");
  } else {
    server.send(400, "text/plain", "Button argument is missing");
  }
}
