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
extern unsigned long mil_LastUserInput;

static byte boundedByte(JsonVariantConst value, byte fallback, byte minimum, byte maximum)
{
  if (!value.is<int>())
    return fallback;

  return constrain(value.as<int>(), minimum, maximum);
}

static void copyStringSetting(char *destination, size_t destinationSize, JsonVariantConst value)
{
  if (value.is<const char *>())
    strlcpy(destination, value.as<const char *>(), destinationSize);
}

static bool updateSettingsFromJson(const String &payload)
{
  JsonDocument document;
  if (deserializeJson(document, payload))
    return false;

  copyStringSetting(Settings.ssid, sizeof(Settings.ssid), document["ssid"]);
  // Password is only ever sent by the client when the user actually typed a new one - a blank/missing value keeps the stored password
  if (document["pass"].is<const char *>() && strlen(document["pass"].as<const char *>()) > 0)
    strlcpy(Settings.pass, document["pass"].as<const char *>(), sizeof(Settings.pass));
  copyStringSetting(Settings.ip, sizeof(Settings.ip), document["ip"]);
  copyStringSetting(Settings.gateway, sizeof(Settings.gateway), document["gateway"]);

  Settings.VolumeSteps = boundedByte(document["VolumeSteps"], Settings.VolumeSteps, 10, 255);
  Settings.MinAttenuation = boundedByte(document["MinAttenuation"], Settings.MinAttenuation, 0, 254);
  Settings.MaxAttenuation = boundedByte(document["MaxAttenuation"], Settings.MaxAttenuation, Settings.MinAttenuation + 1, 255);
  Settings.MaxStartVolume = boundedByte(document["MaxStartVolume"], Settings.MaxStartVolume, 0, Settings.VolumeSteps);
  Settings.MuteLevel = boundedByte(document["MuteLevel"], Settings.MuteLevel, 0, Settings.VolumeSteps);
  if (document["RecallSetLevel"].is<bool>())
    Settings.RecallSetLevel = document["RecallSetLevel"].as<bool>();
  if (document["ExtPowerRelayTrigger"].is<bool>())
    Settings.ExtPowerRelayTrigger = document["ExtPowerRelayTrigger"].as<bool>();

  JsonArrayConst inputs = document["Input"].as<JsonArrayConst>();
  for (byte index = 0; index < 5 && index < inputs.size(); index++)
  {
    JsonObjectConst input = inputs[index];
    Settings.Input[index].Active = boundedByte(input["Active"], Settings.Input[index].Active, INPUT_HT_PASSTHROUGH, INPUT_INACTIVATED);
    copyStringSetting(Settings.Input[index].Name, sizeof(Settings.Input[index].Name), input["Name"]);
    Settings.Input[index].MinVol = boundedByte(input["MinVol"], Settings.Input[index].MinVol, 0, Settings.VolumeSteps);
    Settings.Input[index].MaxVol = boundedByte(input["MaxVol"], Settings.Input[index].MaxVol, Settings.Input[index].MinVol, Settings.VolumeSteps);
    Settings.Input[index].Gain = boundedByte(input["Gain"], Settings.Input[index].Gain, 0, 255);
  }

  Settings.Trigger1Active = boundedByte(document["Trigger1Active"], Settings.Trigger1Active, 0, 1);
  Settings.Trigger1Type = boundedByte(document["Trigger1Type"], Settings.Trigger1Type, 0, 1);
  Settings.Trigger1OnDelay = boundedByte(document["Trigger1OnDelay"], Settings.Trigger1OnDelay, 0, 255);
  Settings.Trigger2Active = boundedByte(document["Trigger2Active"], Settings.Trigger2Active, 0, 1);
  Settings.Trigger2Type = boundedByte(document["Trigger2Type"], Settings.Trigger2Type, 0, 1);
  Settings.Trigger2OnDelay = boundedByte(document["Trigger2OnDelay"], Settings.Trigger2OnDelay, 0, 255);
  Settings.TriggerInactOffTimer = boundedByte(document["TriggerInactOffTimer"], Settings.TriggerInactOffTimer, 0, 255);
  if (document["ScreenSaverActive"].is<bool>())
    Settings.ScreenSaverActive = document["ScreenSaverActive"].as<bool>();
  Settings.DisplayOnLevel = boundedByte(document["DisplayOnLevel"], Settings.DisplayOnLevel, 0, 3);
  Settings.DisplayDimLevel = boundedByte(document["DisplayDimLevel"], Settings.DisplayDimLevel, 0, 3);
  Settings.DisplayTimeout = boundedByte(document["DisplayTimeout"], Settings.DisplayTimeout, 0, 255);
  Settings.DisplayVolume = boundedByte(document["DisplayVolume"], Settings.DisplayVolume, 0, 2);
  if (document["DisplaySelectedInput"].is<bool>())
    Settings.DisplaySelectedInput = document["DisplaySelectedInput"].as<bool>();
  Settings.DisplayTemperature1 = boundedByte(document["DisplayTemperature1"], Settings.DisplayTemperature1, 0, 3);
  Settings.DisplayTemperature2 = boundedByte(document["DisplayTemperature2"], Settings.DisplayTemperature2, 0, 3);

  writeSettingsToEEPROM();
  return true;
}

static String remoteStateAsJson()
{
  JsonDocument document;
  document["selectedInput"] = RuntimeSettings.CurrentInput;
  document["volume"] = RuntimeSettings.CurrentVolume;
  document["muted"] = RuntimeSettings.Muted;
  document["standby"] = appMode == APP_STANDBY_MODE;

  JsonArray inputs = document["inputs"].to<JsonArray>();
  for (byte index = 0; index < 5; index++)
  {
    JsonObject input = inputs.add<JsonObject>();
    input["name"] = Settings.Input[index].Name;
    input["active"] = Settings.Input[index].Active != INPUT_INACTIVATED;
    input["minVolume"] = Settings.Input[index].MinVol;
    input["maxVolume"] = Settings.Input[index].MaxVol;
  }

  String output;
  serializeJson(document, output);
  return output;
}

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
  String ssid = Settings.ssid;
  String ip = Settings.ip;
  String gateway = Settings.gateway;
  ssid.trim();
  ip.trim();
  gateway.trim();

  if (ssid.length() == 0)
  {
    debugln("Undefined SSID.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (ip.length() != 0 || gateway.length() != 0)
  {
    if (ip.length() == 0 || gateway.length() == 0 ||
        !localIP.fromString(ip) || !localGateway.fromString(gateway))
    {
      debugln("Invalid static IP configuration.");
      return false;
    }

    if (!WiFi.config(localIP, localGateway, subnet))
    {
      debugln("STA failed to configure static IP.");
      return false;
    }
  }

  WiFi.begin(ssid.c_str(), Settings.pass);
  debug("Connecting to WiFi... "); debugln(ssid);

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
            { request->send(SPIFFS, "/remote.html", "text/html"); });
  server.on("/remote", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/remote.html", "text/html"); });

  server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "application/json", exportSettingsAsJson()); });
  server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              if (!request->hasParam("settings", true) || !updateSettingsFromJson(request->getParam("settings", true)->value()))
              {
                request->send(400, "application/json", "{\"error\":\"Invalid settings\"}");
                return;
              }

              request->send(200, "application/json", "{\"ok\":true}");
            });
  server.on("/api/remote", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "application/json", remoteStateAsJson()); });
  server.on("/api/remote", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              mil_LastUserInput = millis();

              if (request->hasParam("power", true))
              {
                if (request->getParam("power", true)->value() != "toggle")
                {
                  request->send(400, "application/json", "{\"error\":\"Invalid power command\"}");
                  return;
                }
                requestPowerToggle();
              }

              if (request->hasParam("input", true))
              {
                int input = request->getParam("input", true)->value().toInt();
                if (input < 0 || input > 4 || !setInput(input))
                {
                  request->send(400, "application/json", "{\"error\":\"Invalid input\"}");
                  return;
                }
              }

              if (request->hasParam("mute", true))
              {
                const String &muteValue = request->getParam("mute", true)->value();
                if (muteValue != "toggle")
                {
                  request->send(400, "application/json", "{\"error\":\"Invalid mute command\"}");
                  return;
                }
                if (RuntimeSettings.Muted)
                  unmuteOutput();
                else
                  muteOutput();
              }

              if (request->hasParam("volume", true))
              {
                int volume = request->getParam("volume", true)->value().toInt();
                byte selectedInput = RuntimeSettings.CurrentInput;
                if (volume < Settings.Input[selectedInput].MinVol || volume > Settings.Input[selectedInput].MaxVol)
                {
                  request->send(400, "application/json", "{\"error\":\"Invalid volume\"}");
                  return;
                }
                setVolume(volume);
              }

              request->send(200, "application/json", remoteStateAsJson());
            });

  server.on("/INPUT1", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); request->send(200, "text/plain", String(setInput(0))); });
  server.on("/INPUT2", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); request->send(200, "text/plain", String(setInput(1))); });
  server.on("/INPUT3", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); request->send(200, "text/plain", String(setInput(2))); });
  server.on("/INPUT4", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); request->send(200, "text/plain", String(setInput(3))); });
  server.on("/INPUT5", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); request->send(200, "text/plain", String(setInput(4))); });

  server.on("/MUTE", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); muteOutput(); request->send(200, "text/plain", "Mute"); });
  server.on("/UNMUTE", HTTP_GET, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); unmuteOutput(); request->send(200, "text/plain", "Unmute"); });

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
