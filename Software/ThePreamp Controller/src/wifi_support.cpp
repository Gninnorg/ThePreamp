#include "wifi_support.h"
#include "controller_config.h"
#include "debug.h"
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

extern IPAddress localIP;
extern IPAddress localGateway;
extern IPAddress subnet;
extern unsigned long previousMillis;
extern const long interval;

void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    debugln("An error has occurred while mounting SPIFFS");
  }
  debugln("SPIFFS mounted successfully");
}

bool initWiFi()
{ 
  if (strlen(Settings.ssid) == 0 || strlen(Settings.ip) == 0)
  {
    debugln("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  localIP.fromString(Settings.ip);
  localGateway.fromString(Settings.gateway);

  if (!WiFi.config(localIP, localGateway, subnet))
  {
    debugln("STA Failed to configure");
    return false;
  }

  WiFi.begin(Settings.ssid, Settings.pass);
  debug("Connecting to WiFi... "); debugln(Settings.ssid);

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while (WiFi.status() != WL_CONNECTED)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      debugln("Failed to connect.");
      return false;
    }
  }

  debug("Connected to WiFi. IP: "); debugln(WiFi.localIP());
  return true;
}
