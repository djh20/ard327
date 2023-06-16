#include <Arduino.h>
#include <mcp_can.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ElegantOTA.h>
#include "config.h"

ESP8266WebServer webServer(80);
WiFiServer socketServer(35000);
WiFiClient client;

#define CAN_CS_PIN D8
#define CAN_INT_PIN D2
#define WIFI_SCAN_INTERVAL 30000U

IPAddress local(192, 168, 0, 10);
IPAddress subnet(255, 255, 255, 0); 

uint32_t nextScanMillis = 0;
uint32_t nextMsgMillis = 0;
MCP_CAN can(CAN_CS_PIN);

long unsigned int frameId;
unsigned char frameLength = 0;
unsigned char frameData[8];

bool monitoring = false;
long unsigned int mask = 0;
long unsigned int filter = 0;

void setup() {
  Serial.begin(9600);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  WiFi.softAPConfig(local, local, subnet);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, 1, 1);

  if(can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
  }

  can.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);

  ElegantOTA.begin(&webServer);
  webServer.begin();
  socketServer.begin();
}

int hexToInt(String str)
{
  return (int) strtol(str.c_str(), 0, 16);
}

void loop() {
  uint32_t now = millis();
  
  // Reconnect if device doesn't have IP
  if (WiFi.isConnected() && !WiFi.localIP().isSet())
  {
    WiFi.disconnect();
  }

  if (now >= nextScanMillis && !WiFi.isConnected())
  {
    WiFi.scanNetworks(true, true, 0U, (uint8_t*) WIFI_HOME_SSID);
    nextScanMillis = now + WIFI_SCAN_INTERVAL;
  }

  int8_t totalNetworks = WiFi.scanComplete();
  if (totalNetworks > 0)
  {
    Serial.print("Connecting to ");
    Serial.println(WIFI_HOME_SSID);
    WiFi.begin(WIFI_HOME_SSID, WIFI_HOME_PASS);
    WiFi.scanDelete();
  }

  if (!client.connected()) {
    client.stop();
    client = socketServer.accept();
  }

  if (client.connected() && client.available() > 0) {
    String data = client.readStringUntil('\r');

    if (monitoring) {
      monitoring = false;
      client.print("STOPPED\r>");

    } else if (data.startsWith("ATCM")) {
      String hex = data.substring(data.length() - 3);
      mask = hexToInt(hex);
      client.print("OK\r>");

    } else if (data.startsWith("ATCF")) {
      String hex = data.substring(data.length() - 3);
      filter = hexToInt(hex);
      client.print("OK\r>");

    } else if (data.startsWith("ATCRA")) {
      String hex = data.substring(data.length() - 3);
      filter = hexToInt(hex);
      mask = 0x7FF;
      client.print("OK\r>");

    } else if (data.startsWith("ATMA")) {
      monitoring = true;

    } else if (data.startsWith("TEST")) {
      byte txData[] = {0x03, 0x22, 0x12, 0x30};
      if (can.sendMsgBuf(0x792, 4, txData) == CAN_OK) {
        client.print("SENT\r");
        monitoring = true;
        mask = 0x7FF;
        filter = 0x793;
      } else {
        client.print("ERROR\r>");
      }

    } else {
      client.print("?\r>");
    }
  }

  /*
  if (!digitalRead(CAN_INT_PIN)) {
    can.readMsgBuf(&frameId, &frameLength, frameData);

    if (monitoring && client.connected() && (frameId & mask) == filter) {
      client.flush();
      
      client.printf("%.3lX", frameId);
      for (byte i = 0; i < frameLength; i++) {
        client.printf("%.2X", frameData[i]);
      }
      client.print('\r');
    }
  }
  */

  if (now >= nextMsgMillis)
  {
    if (monitoring && client.connected()) {
      //client.flush();
      client.print("60D");
      client.print("00");
      client.print("FF");
      client.print("00");
      client.print("FF");
      client.print("00");
      client.print("FF");
      client.print("00");
      client.print("FF");
      client.print("\r");
    }
    nextMsgMillis = now + 10;
  }

  webServer.handleClient();
}
