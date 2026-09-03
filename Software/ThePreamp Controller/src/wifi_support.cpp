#include "wifi_support.h"
#include "controller_config.h"
#include "debug.h"
#include "audio_controller.h"
#include "display_controller.h"
#include "input_controller.h"
#include "wifi_QR.h"
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>

extern AsyncWebServer server;
extern DNSServer dnsServer;
extern IPAddress localIP;
extern IPAddress localGateway;
extern IPAddress subnet;
extern U8G2_SH1122_256X64_F_4W_HW_SPI left_display;
extern U8G2_SH1122_256X64_F_4W_HW_SPI right_display;
extern const char *PARAM_INPUT_1;
extern const char *PARAM_INPUT_2;
extern const char *PARAM_INPUT_3;
extern const char *PARAM_INPUT_4;

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

static void setupNormalModeServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.on("/INPUT1", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(setInput(0))); });
  server.on("/INPUT2", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(setInput(1))); });
  server.on("/INPUT3", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(setInput(2))); });
  server.on("/INPUT4", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(setInput(3))); });
  server.on("/INPUT5", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(setInput(4))); });

  server.on("/MUTE", HTTP_GET, [](AsyncWebServerRequest *request)
            { muteOutput(); request->send(200, "text/plain", "Mute"); });
  server.on("/UNMUTE", HTTP_GET, [](AsyncWebServerRequest *request)
            { unmuteOutput(); request->send(200, "text/plain", "Unmute"); });

  server.serveStatic("/", SPIFFS, "/");
  ElegantOTA.begin(&server);
  WebSerial.begin(&server);
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    debug("Received ");
    debug(len);
    debugln(" bytes from WebSerial: ");
    Serial.write(data, len);

    String input;
    for (size_t i = 0; i < len; i++)
      input += char(data[i]);

    String command;
    String value;
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0)
    {
      command = input.substring(0, spaceIndex);
      value = input.substring(spaceIndex + 1);
    }
    else
    {
      command = input;
    }
    command.trim();
    value.trim();

    WebSerial.println("Received Data...");
    WebSerial.print("Command: ");
    WebSerial.println(command);
    WebSerial.print("Value: ");
    WebSerial.println(value);

    if (command == "HELP")
    {
      WebSerial.println("IR_UP value");
      WebSerial.println("IR_DOWN value");
    }
    else if (command == "EXPORT-SETTINGS")
    {
      WebSerial.println(exportSettingsAsJson());
    }
  });
  server.begin();
}

static void setupAccessPointServer()
{
  debugln("Setting AP (Access Point)");
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.softAP("ThePreAmp", NULL, 6, 0);
  dnsServer.start(53, "*", WiFi.softAPIP());

  debug("AP IP address: ");
  debugln(WiFi.softAPIP());

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/style.css.gz", "text/css");
              response->addHeader("Content-Encoding", "gzip");
              request->send(response); });
  server.on("update.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/update.html", "text/html"); });
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(SPIFFS, "/wifi.html", "text/html"); });

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    for (size_t i = 0; i < request->params(); i++)
    {
      const AsyncWebParameter *parameter = request->getParam(i);
      if (!parameter->isPost())
        continue;
      if (parameter->name() == PARAM_INPUT_1)
        strcpy(Settings.ssid, parameter->value().c_str());
      if (parameter->name() == PARAM_INPUT_2)
        strcpy(Settings.pass, parameter->value().c_str());
      if (parameter->name() == PARAM_INPUT_3)
        strcpy(Settings.ip, parameter->value().c_str());
      if (parameter->name() == PARAM_INPUT_4)
        strcpy(Settings.gateway, parameter->value().c_str());
    }
    writeSettingsToEEPROM();
    request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + String(Settings.ip));
    debugln("Restarting...");
    delay(3000);
    ESP.restart();
  });

  ElegantOTA.begin(&server);
  server.begin();

  left_display.clearBuffer();
  left_display.drawXBMP(0, 0, 64, 64, ThePreAmp_wifi_QR);
  left_display.setFont(u8g2_font_luBS18_tf);
  left_display.drawStr(74, 31, "Scan to");
  left_display.drawStr(74, 58, "setup WiFi");
  left_display.sendBuffer();

  right_display.clearBuffer();
  right_display.setFont(u8g2_font_luBS18_tf);
  right_display.drawStr(0, 31, "Push volume");
  right_display.drawStr(0, 58, "button to skip");
  right_display.sendBuffer();

  while (getUserCommand() != KEY_SELECT)
  {
    ElegantOTA.loop();
    dnsServer.processNextRequest();
  }
}

void startWiFiSupport()
{
  initSPIFFS();
  if (initWiFi())
    setupNormalModeServer();
  else
    setupAccessPointServer();
}
