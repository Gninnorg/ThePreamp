#include "wifi_support.h"
#include "controller_config.h"
#include "debug.h"
#include "audio_controller.h"
#include "display_controller.h"
#include "input_controller.h"
#include "wifi_QR.h"
#include "mqtt_controller.h"
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>

// Wi-Fi and web support. Overview:
//  - startWiFiSupport() mounts SPIFFS (web pages in data/), then tries to join the configured router.
//  - On success: setupNormalModeServer() serves the remote/settings web UI and the JSON API.
//  - On failure: setupAccessPointServer() starts the "ThePreAmp" access point with a Wi-Fi setup page.
// IMPORTANT: web handlers run on the AsyncTCP task, not in loop(). They must NOT touch the
// attenuator, inputs or displays (shared SPI bus). Instead they set requestedInput/requestedVolume/
// requestedMute, which loop() applies in processPendingRequests() (see main.cpp).

// Globals owned by main.cpp
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

// ---- JSON helpers for the settings API ----

// Reads an integer from JSON and clamps it to [minimum, maximum]; returns fallback if missing/not a number
static byte boundedByte(JsonVariantConst value, byte fallback, byte minimum, byte maximum)
{
  if (!value.is<int>())
    return fallback;

  return constrain(value.as<int>(), minimum, maximum);
}

// Copies a JSON string into a fixed-size buffer (truncating); ignores non-string values
static void copyStringSetting(char *destination, size_t destinationSize, JsonVariantConst value)
{
  if (value.is<const char *>())
    strlcpy(destination, value.as<const char *>(), destinationSize);
}

// Applies a settings JSON (from the web settings page) to Settings and saves it to EEPROM.
// Every value is range-checked; missing or invalid fields keep their current value.
// Returns false only if the payload isn't valid JSON.
static bool updateSettingsFromJson(const String &payload)
{
  JsonDocument document;
  if (deserializeJson(document, payload))
    return false;

  // Network settings
  copyStringSetting(Settings.ssid, sizeof(Settings.ssid), document["ssid"]);
  // Password is only ever sent by the client when the user actually typed a new one - a blank/missing value keeps the stored password
  if (document["pass"].is<const char *>() && strlen(document["pass"].as<const char *>()) > 0)
    strlcpy(Settings.pass, document["pass"].as<const char *>(), sizeof(Settings.pass));
  copyStringSetting(Settings.ip, sizeof(Settings.ip), document["ip"]);
  copyStringSetting(Settings.gateway, sizeof(Settings.gateway), document["gateway"]);

  // Volume and attenuation limits (order matters: later limits depend on earlier ones)
  Settings.VolumeSteps = boundedByte(document["VolumeSteps"], Settings.VolumeSteps, 10, 255);
  Settings.MinAttenuation = boundedByte(document["MinAttenuation"], Settings.MinAttenuation, 0, 254);
  Settings.MaxAttenuation = boundedByte(document["MaxAttenuation"], Settings.MaxAttenuation, Settings.MinAttenuation + 1, 255);
  Settings.MaxStartVolume = boundedByte(document["MaxStartVolume"], Settings.MaxStartVolume, 0, Settings.VolumeSteps);
  Settings.MuteLevel = boundedByte(document["MuteLevel"], Settings.MuteLevel, 0, Settings.VolumeSteps);
  if (document["RecallSetLevel"].is<bool>())
    Settings.RecallSetLevel = document["RecallSetLevel"].as<bool>();
  if (document["ExtPowerRelayTrigger"].is<bool>())
    Settings.ExtPowerRelayTrigger = document["ExtPowerRelayTrigger"].as<bool>();

  // Per-input settings (5 inputs): mode, name, volume range and gain
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

  // 12V trigger outputs
  Settings.Trigger1Active = boundedByte(document["Trigger1Active"], Settings.Trigger1Active, 0, 1);
  Settings.Trigger1Type = boundedByte(document["Trigger1Type"], Settings.Trigger1Type, 0, 1);
  Settings.Trigger1OnDelay = boundedByte(document["Trigger1OnDelay"], Settings.Trigger1OnDelay, 0, 255);
  Settings.Trigger2Active = boundedByte(document["Trigger2Active"], Settings.Trigger2Active, 0, 1);
  Settings.Trigger2Type = boundedByte(document["Trigger2Type"], Settings.Trigger2Type, 0, 1);
  Settings.Trigger2OnDelay = boundedByte(document["Trigger2OnDelay"], Settings.Trigger2OnDelay, 0, 255);
  Settings.TriggerInactOffTimer = boundedByte(document["TriggerInactOffTimer"], Settings.TriggerInactOffTimer, 0, 255);
  // Display behaviour
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

// ---- IR remote learning helpers ----
// Flow: the web UI starts learning, loop() captures a code into irLearnCode, the UI polls until
// irLearnCodeReady, then assigns the code to a named command (IR_UP, IR_MUTE, ...).

// Maps a command name from the web UI to the matching stored IR code, or nullptr if unknown
static uint64_t *irCodeField(const String &command)
{
  if (command == "IR_ON")
    return &Settings.IR_ON;
  if (command == "IR_OFF")
    return &Settings.IR_OFF;
  if (command == "IR_UP")
    return &Settings.IR_UP;
  if (command == "IR_DOWN")
    return &Settings.IR_DOWN;
  if (command == "IR_LEFT")
    return &Settings.IR_LEFT;
  if (command == "IR_RIGHT")
    return &Settings.IR_RIGHT;
  if (command == "IR_SELECT")
    return &Settings.IR_SELECT;
  if (command == "IR_INFO")
    return &Settings.IR_INFO;
  if (command == "IR_MUTE")
    return &Settings.IR_MUTE;
  if (command == "IR_POWER")
    return &Settings.IR_POWER;
  if (command == "IR_1")
    return &Settings.IR_1;
  if (command == "IR_2")
    return &Settings.IR_2;
  if (command == "IR_3")
    return &Settings.IR_3;
  if (command == "IR_4")
    return &Settings.IR_4;
  if (command == "IR_5")
    return &Settings.IR_5;
  return nullptr;
}

// Stores the most recently learned IR code for the given command and saves to EEPROM
static bool applyLearnedIrCode(const String &command)
{
  if (!irLearnCodeReady)
    return false;

  uint64_t *field = irCodeField(command);
  if (!field)
    return false;

  *field = irLearnCode;
  writeSettingsToEEPROM();
  irLearnCodeReady = false;
  return true;
}

// Un-assigns the IR code for the given command (0 = none)
static bool clearIrCode(const String &command)
{
  uint64_t *field = irCodeField(command);
  if (!field)
    return false;

  *field = 0;
  writeSettingsToEEPROM();
  return true;
}

// ---- Request queue helpers ----
// A queued request (>= 0) hasn't been applied by loop() yet. The "effective" getters return the
// queued value if there is one, otherwise the real state, so the UI sees its own change immediately
// and repeated requests (e.g. a mute toggle) build on the pending state.

static bool effectiveMuted()
{
  return requestedMute >= 0 ? requestedMute == 1 : RuntimeSettings.Muted;
}

static byte effectiveInput()
{
  return requestedInput >= 0 ? requestedInput : RuntimeSettings.CurrentInput;
}

// Validates and queues an input change for loop() to carry out; the web task never touches the hardware itself
static bool queueInput(int input)
{
  if (input < 0 || input > 4 || appMode != APP_NORMAL_MODE || Settings.Input[input].Active == INPUT_INACTIVATED)
    return false;
  requestedInput = input;
  return true;
}

// State for the web Remote (polled every 3 s by remote.html): input, volume, mute, standby and input list
static String remoteStateAsJson()
{
  JsonDocument document;
  document["selectedInput"] = effectiveInput();
  document["volume"] = requestedVolume >= 0 ? requestedVolume : RuntimeSettings.CurrentVolume;
  document["muted"] = effectiveMuted();
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

// Mounts the SPIFFS filesystem that holds the web pages (formats it if mounting fails)
void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    debugln("An error has occurred while mounting SPIFFS");
  }
  debugln("SPIFFS mounted successfully");
}

// Joins the configured router as a station (static IP if ip+gateway are set, otherwise DHCP).
// Blocks until connected or `interval` ms have passed. Returns false on missing/invalid config or timeout.
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

// Web server used when connected to the router: remote UI, JSON API, legacy endpoints, OTA and WebSerial
static void setupNormalModeServer()
{
  // --- Pages ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/remote.html", "text/html"); });
  server.on("/remote", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/remote.html", "text/html"); });

  // --- Settings API: GET returns all settings as JSON, POST ("settings" form field) updates and saves them ---
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
  // --- Remote API: GET returns current state; POST accepts power=toggle, input=<0-4>, mute=toggle, volume=<n>.
  // Changes are only queued here; loop() carries them out. Invalid values get a 400 response. ---
  server.on("/api/remote", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "application/json", remoteStateAsJson()); });
  server.on("/api/remote", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              mil_LastUserInput = millis(); // counts as user activity (keeps the display awake)

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
                if (!queueInput(input))
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
                // Ignored in standby; otherwise flip the (possibly still queued) mute state
                if (appMode == APP_NORMAL_MODE)
                  requestedMute = effectiveMuted() ? 0 : 1;
              }

              if (request->hasParam("volume", true))
              {
                int volume = request->getParam("volume", true)->value().toInt();
                // The allowed volume range depends on the input (which may itself be queued)
                byte selectedInput = effectiveInput();
                if (volume < Settings.Input[selectedInput].MinVol || volume > Settings.Input[selectedInput].MaxVol)
                {
                  request->send(400, "application/json", "{\"error\":\"Invalid volume\"}");
                  return;
                }
                if (appMode == APP_NORMAL_MODE)
                  requestedVolume = volume;
              }

              request->send(200, "application/json", remoteStateAsJson());
            });

  // --- MQTT API: GET returns the config and connection state; POST updates it (blank password keeps the old one) ---
  server.on("/api/mqtt", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              const MqttConfig &config = mqttGetConfig();
              JsonDocument document;
              document["enabled"] = config.enabled;
              document["host"] = config.host;
              document["port"] = config.port;
              document["user"] = config.user;
              document["baseTopic"] = config.baseTopic;
              document["connected"] = mqttIsConnected();
              String output;
              serializeJson(document, output);
              request->send(200, "application/json", output);
            });
  server.on("/api/mqtt", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              MqttConfig config = mqttGetConfig();
              if (request->hasParam("enabled", true))
                config.enabled = request->getParam("enabled", true)->value() == "true";
              if (request->hasParam("host", true))
                strlcpy(config.host, request->getParam("host", true)->value().c_str(), sizeof(config.host));
              if (request->hasParam("port", true))
              {
                int port = request->getParam("port", true)->value().toInt();
                if (port < 1 || port > 65535)
                {
                  request->send(400, "application/json", "{\"error\":\"Invalid port\"}");
                  return;
                }
                config.port = port;
              }
              if (request->hasParam("user", true))
                strlcpy(config.user, request->getParam("user", true)->value().c_str(), sizeof(config.user));
              // A blank password keeps the stored one
              if (request->hasParam("pass", true) && request->getParam("pass", true)->value().length() > 0)
                strlcpy(config.pass, request->getParam("pass", true)->value().c_str(), sizeof(config.pass));
              if (request->hasParam("baseTopic", true))
                strlcpy(config.baseTopic, request->getParam("baseTopic", true)->value().c_str(), sizeof(config.baseTopic));
              mqttSetConfig(config);
              request->send(200, "application/json", "{\"ok\":true}");
            });
  // --- IR learning API: start -> poll until ready -> apply to a command (or cancel / clear a command) ---
  server.on("/api/ir-learn/start", HTTP_POST, [](AsyncWebServerRequest *request)
            { mil_LastUserInput = millis(); startIrLearning(); request->send(200, "application/json", "{\"learning\":true}"); });
  server.on("/api/ir-learn/cancel", HTTP_POST, [](AsyncWebServerRequest *request)
            { irLearnActive = false; irLearnCodeReady = false; request->send(200, "application/json", "{\"ok\":true}"); });
  server.on("/api/ir-learn/poll", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              JsonDocument document;
              document["learning"] = irLearnActive;
              document["ready"] = irLearnCodeReady;
              if (irLearnCodeReady)
                document["code"] = String(irLearnCode);

              String output;
              serializeJson(document, output);
              request->send(200, "application/json", output);
            });
  server.on("/api/ir-learn/apply", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              mil_LastUserInput = millis();
              if (!request->hasParam("command", true) || !applyLearnedIrCode(request->getParam("command", true)->value()))
              {
                request->send(400, "application/json", "{\"error\":\"No learned code to apply\"}");
                return;
              }

              request->send(200, "application/json", "{\"ok\":true}");
            });
  server.on("/api/ir-learn/clear", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              mil_LastUserInput = millis();
              if (!request->hasParam("command", true) || !clearIrCode(request->getParam("command", true)->value()))
              {
                request->send(400, "application/json", "{\"error\":\"Unknown command\"}");
                return;
              }

              request->send(200, "application/json", "{\"ok\":true}");
            });

  // --- Legacy endpoints kept for simple HTTP automation: /INPUT1../INPUT5 (returns 1 if accepted, 0 if not), /MUTE, /UNMUTE ---
  for (int index = 0; index < 5; index++)
  {
    String path = "/INPUT" + String(index + 1);
    server.on(path.c_str(), HTTP_GET, [index](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(queueInput(index))); });
  }

  server.on("/MUTE", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (appMode == APP_NORMAL_MODE)
                requestedMute = 1;
              request->send(200, "text/plain", "Mute"); });
  server.on("/UNMUTE", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (appMode == APP_NORMAL_MODE)
                requestedMute = 0;
              request->send(200, "text/plain", "Unmute"); });

  // Anything else is served as a static file from SPIFFS; unknown paths get a plain 404
  server.serveStatic("/", SPIFFS, "/");
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(404, "text/plain", "Not found"); });

  // Firmware/filesystem upload page (/update) and browser serial console (/webserial)
  ElegantOTA.begin(&server);
  WebSerial.begin(&server);
  // Text commands typed in the WebSerial console: "<COMMAND> [value]"
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

// Fallback when no router connection is possible: opens the "ThePreAmp" access point with a captive
// portal. The user enters SSID/password (and optional static IP) on wifi.html, which are saved before
// restarting. Blocks until the user presses the volume button to skip setup.
static void setupAccessPointServer()
{
  debugln("Setting AP (Access Point)");
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.softAP("ThePreAmp", NULL, 6, 0);
  dnsServer.start(53, "*", WiFi.softAPIP());

  debug("AP IP address: ");
  debugln(WiFi.softAPIP());

  server.on("update.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/update.html", "text/html"); });
  // Captive portal: every unknown URL (and the DNS catch-all above) shows the setup page
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(SPIFFS, "/wifi.html", "text/html"); });

  // Form submit: store the four Wi-Fi fields (matched by form field name), save, and reboot into normal mode
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

  // Show setup instructions: QR code on the left display, skip hint on the right
  // (safe to draw here: loop() hasn't started yet, so nothing else uses the SPI bus)
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

  // Wait here until the volume button is pressed; keep OTA and the captive-portal DNS alive meanwhile
  while (getUserCommand() != KEY_SELECT)
  {
    ElegantOTA.loop();
    dnsServer.processNextRequest();
  }
}

// Entry point called from setup(): mount files, then choose normal (router) or access-point mode
void startWiFiSupport()
{
  initSPIFFS();
  if (initWiFi())
    setupNormalModeServer();
  else
    setupAccessPointServer();
}
