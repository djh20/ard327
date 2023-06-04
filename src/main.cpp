#include <Arduino.h>
#include <SoftwareSerial.h>
#include <mcp_can.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ElegantOTA.h>
#include "config.h"

ESP8266WebServer server(80);
SoftwareSerial btSerial(D3, D4);

#define CAN_CS_PIN D8
#define CAN_INT_PIN D2
#define WIFI_SCAN_INTERVAL 30000U

uint32_t nextScanMillis = 0;
MCP_CAN can(CAN_CS_PIN);

long unsigned int frameId;
unsigned char frameLength = 0;
unsigned char frameData[8];

bool monitoring = false;
long unsigned int mask = 0;
long unsigned int filter = 0;

void setup() {
  Serial.begin(9600);
  btSerial.begin(115200);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  if(can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  can.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);

  ElegantOTA.begin(&server);
  server.begin();
}

int hexToInt(String str)
{
  return (int) strtol(str.c_str(), 0, 16);
}

void setFilter(uint32_t filter) {
  can.init_Filt(0, 0, filter);
}

void setMask(uint32_t mask) {
  can.init_Mask(0, 0, mask);
  can.init_Mask(1, 0, mask);
}

void loop() {
  /*
  if (Serial.available()) {
    btSerial.write(Serial.read());
  }
  */
  
  uint32_t now = millis();
  
  if (WiFi.isConnected() && !WiFi.localIP().isSet())
  {
    // Reconnect if device doesn't have IP
    WiFi.disconnect();
  }

  if (now >= nextScanMillis && !WiFi.isConnected())
  {
    WiFi.scanNetworks(true, true, 0U, (uint8_t*) WIFI_SSID);
    nextScanMillis = now + WIFI_SCAN_INTERVAL;
  }

  int8_t totalNetworks = WiFi.scanComplete();
  if (totalNetworks > 0)
  {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.scanDelete();
  }

  if (btSerial.available()) {
    String data = btSerial.readStringUntil('\r');
    //btSerial.println(data);

    //Serial.print("RX: ");
    //Serial.println(data);

    if (monitoring) {
      monitoring = false;
      btSerial.print("STOPPED\r>");

    } else if (data.startsWith("ATCM")) {
      String hex = data.substring(data.length() - 3);
      mask = hexToInt(hex);
      //setMask(hexToInt(hex) << 16);

      btSerial.print("OK\r>");

    } else if (data.startsWith("ATCF")) {
      String hex = data.substring(data.length() - 3);
      filter = hexToInt(hex);
      //setFilter(hexToInt(hex) << 16);

      //Serial.print("FILTER: "); Serial.println(hex);
      btSerial.print("OK\r>");

    } else if (data.startsWith("ATCRA")) {
      String hex = data.substring(data.length() - 3);
      filter = hexToInt(hex);
      mask = 0x7FF;
      //setFilter(hexToInt(hex) << 16);
      //setMask(0x07FF0000);

      btSerial.print("OK\r>");

    } else if (data.startsWith("ATMA")) {
      monitoring = true;

    } else if (data.startsWith("TEST")) {
      byte txData[] = {0x03, 0x22, 0x12, 0x30};
      if (can.sendMsgBuf(0x792, 4, txData) == CAN_OK) {
        btSerial.print("SENT\r");
        monitoring = true;
        mask = 0x7FF;
        filter = 0x793;
      } else {
        btSerial.print("ERROR\r>");
      }
      
      //btSerial.print("OK\r>");
      
    } else {
      btSerial.print("?\r>");
    }
  }

  if (!digitalRead(CAN_INT_PIN)) {
    can.readMsgBuf(&frameId, &frameLength, frameData);

    if (monitoring && (frameId & mask) == filter) {
    //if (monitoring) {
      btSerial.printf("%.3lX", frameId);
      for (byte i = 0; i < frameLength; i++) {
        btSerial.printf("%.2X", frameData[i]);
      }
      btSerial.print('\r');
    }
  }

  server.handleClient();
}
