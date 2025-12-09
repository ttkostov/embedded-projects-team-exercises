#include <Arduino.h>
#include "LittleFS.h"

#include <ESP8266WiFi.h>      // Library for Wi-Fi functionality
#include <ESP8266WebServer.h> // Library to create and manage a web server
#include <FS.h>               // Library for working with file systems (SPIFFS)

#include <string.h>

void handleNotFound();
void handleMove(int distance);
void handleCompass();
void handleFind(String direction);
void handleLogs();
void handleStatus();

const char *ssid = "Titenet-IoT";  // Wi-Fi network name (SSID)
const char *password = "7kDtaphg"; // Wi-Fi network password

unsigned long lastMessageReceived = 0;
const unsigned int ACTIVE_INTERVAL = 10000; // 10 seconds

// a queue for saving the logs
#define QUEUE_SIZE 100

String queueBuf[QUEUE_SIZE];
int queueHead = 0;  // next write index
int queueCount = 0; // number of stored messages

// Push message into queue (overwrite oldest if full)
void queuePush(const String &msg)
{
  queueBuf[queueHead] = msg;
  queueHead = (queueHead + 1) % QUEUE_SIZE;

  if (queueCount < QUEUE_SIZE)
    queueCount++;
}

// Return entire queue as a single string (each entry on new line)
String queueToString()
{
  String out = "";

  // Start from the most recently inserted message
  int index = (queueHead - 1 + QUEUE_SIZE) % QUEUE_SIZE;

  for (int i = 0; i < queueCount; i++)
  {
    out += queueBuf[index];
    if (i < queueCount - 1)
      out += "\n";

    // Move backwards, wrap around if needed
    index = (index - 1 + QUEUE_SIZE) % QUEUE_SIZE;
  }

  return out;
}

ESP8266WebServer server(80); // Create an instance of the WebServer on port 80 (default HTTP port)

void setup()
{
  Serial.begin(9600);

  // Initialize the file system (SPIFFS) on the ESP8266
  if (!LittleFS.begin())
  { // Initialize and check success on SPIFFS
    Serial.println("Error while mounting LittleFS");
    return;
  }

  // Connect to the Wi-Fi network using the provided SSID and password
  Serial.print("\nConnecting to " + (String)ssid);
  WiFi.begin(ssid, password); // Begin connecting to Wi-Fi
  while (WiFi.status() != WL_CONNECTED)
  { // Wait in a loop until the connection is successful
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP address: " + WiFi.localIP().toString()); // Print the IP address of the ESP8266 when connected

  // Set up the web pages (URLS) and files that the server will show/use when someone visits the site
  server.serveStatic("/", LittleFS, "/index.html");             // Serves and shows the main webpage (HTML file) when someone visits the home page
  server.serveStatic("/style.css", LittleFS, "/style.css");     // Serve the CSS file for styling
  server.serveStatic("/script.js", LittleFS, "/script.js");     // Serve the JavaScript file
  server.serveStatic("/favicon.ico", LittleFS, "/favicon.png"); // Serve a favicon (small icon) for the website

  // Define custom actions/function calls for specific URLs (ie when a button press from webpage requests a particular URL)
  server.on("/forwards5", []()
            { handleMove(5); }); // When requesting URL "/forwards5", call the handleMove function with parameter 5
  server.on("/forwards20", []()
            { handleMove(20); }); // [](){ handleMove(x); } is a lambda function that contains the code
  server.on("/backwards5", []()
            { handleMove(-5); }); // to be executed when the route is visited. These lambda functions call
  server.on("/backwards20", []()
            { handleMove(-20); });      // handleMove(x) as soon as it's triggered.
  server.on("/compass", handleCompass); // When requesting URL "/compass", call the handleCompass function
  server.on("/findnorth", []()
            { handleFind("north"); }); // When requesting URL "/findnorth", call the handleFind function
  server.on("/status", []()
            { handleStatus(); });
  server.on("/log", []()
            { handleLogs(); });

  //  If someone tries to access a URL that does not exist (e.g. due to a typo), call the handleNotFound function
  server.onNotFound(handleNotFound);

  // Start the web server
  server.begin();
}

void loop()
{
  server.handleClient(); // Listen for incoming HTTP requests and respond to them

  // read mnessages on the serial and save them to a variable
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      lastMessageReceived = millis();
      queuePush(String(millis()) + ": " + line);
    }
  }
}

// This function is called when a non-existing URL is accessed (404 error)
void handleNotFound()
{
  server.send(404, "text/plain", "404: Not Found"); // Send a 404 response with a plain text message
}

// This function handles movement commands like "/forwards5" or "/backwards20"
void handleMove(int distance)
{
  if (server.hasArg("speed"))
  {                                                        // Check if there is a "value" argument in the request URL
    String speedString = server.arg("speed");     // Get the "speed" argument from the URL
    Serial.println("Move:" + String(distance) + " --speed=" + speedString);      // Print the movement distance to the serial monitor for Arduino Mega   
  }
  else {
    Serial.println("Move:" + String(distance));         
  }
  server.send(200);                           // Send a 200 OK response to the client (browser)
}

void handleFind(String direction)
{
  Serial.println("Find:" + String(direction)); // Print the find direction to the serial monitor for Arduino Mega
  server.send(200);                            // Send a 200 OK response to the client (browser)
}

// This function handles the compass command (like "/compass?value=30")
void handleCompass()
{
  if (server.hasArg("value"))
  {                                                        // Check if there is a "value" argument in the request URL
    String valueString = server.arg("value");              // Get the "value" argument from the URL
    Serial.println("Turn:" + valueString + " --relative"); // Print the compass value to the serial monitor
  }
  server.send(200);
}

// Handles status request
void handleStatus()
{
  unsigned long now = millis();
  unsigned long diff = now - lastMessageReceived;

  if (diff > ACTIVE_INTERVAL)
  {
    server.send(200, "text/plain", "false");
  }
  else
  {
    server.send(200, "text/plain", "true");
  }
}

// Handles logs request, gives back the last logs as text
void handleLogs()
{
  server.send(200, "text/plain", queueToString());
}

/*
void setup() {
  Serial.begin(9600);

  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  File file = LittleFS.open("/text.txt", "r");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void loop() {

}
*/
/*
#include <Arduino.h>

#define LED 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("LED is off");
  delay(1000);
}
  */