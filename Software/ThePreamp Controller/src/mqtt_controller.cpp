#include "mqtt_controller.h"
#include "controller_config.h"
#include "audio_controller.h"
#include "display_controller.h"
#include "debug.h"
#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

extern byte appMode;
extern volatile byte requestedPowerCommand;
extern unsigned long mil_LastUserInput;

static WiFiClient wifiClient;
static PubSubClient client(wifiClient);
static Preferences prefs;
static MqttConfig config;
static volatile bool reloadRequested = false;
static bool configLoaded = false;

static String deviceId;
static String statusTopic;
static String stateTopic;
static unsigned long lastReconnectAttempt = 0;
static unsigned long lastStatePublish = 0;
static String lastPublishedState;

static const unsigned long RECONNECT_INTERVAL_MS = 5000;
static const unsigned long STATE_REFRESH_INTERVAL_MS = 60000;

static void loadConfig()
{
  memset(&config, 0, sizeof(config));
  prefs.begin("mqtt", true);
  config.enabled = prefs.getBool("enabled", false);
  prefs.getString("host", config.host, sizeof(config.host));
  config.port = prefs.getUShort("port", 1883);
  prefs.getString("user", config.user, sizeof(config.user));
  prefs.getString("pass", config.pass, sizeof(config.pass));
  String base = prefs.getString("base", "thepreamp");
  strlcpy(config.baseTopic, base.c_str(), sizeof(config.baseTopic));
  prefs.end();
  configLoaded = true;
}

const MqttConfig &mqttGetConfig()
{
  if (!configLoaded)
    loadConfig();
  return config;
}

void mqttSetConfig(const MqttConfig &newConfig)
{
  prefs.begin("mqtt", false);
  prefs.putBool("enabled", newConfig.enabled);
  prefs.putString("host", newConfig.host);
  prefs.putUShort("port", newConfig.port);
  prefs.putString("user", newConfig.user);
  prefs.putString("pass", newConfig.pass);
  prefs.putString("base", newConfig.baseTopic);
  prefs.end();
  config = newConfig;
  configLoaded = true;
  mqttRequestReload();
}

void mqttRequestReload()
{
  reloadRequested = true;
}

bool mqttIsConnected()
{
  return client.connected();
}

// Name shown in Home Assistant for an input; falls back to "Input N" and keeps names unique
static String inputLabel(byte index)
{
  char name[sizeof(Settings.Input[index].Name) + 1];
  memcpy(name, Settings.Input[index].Name, sizeof(Settings.Input[index].Name));
  name[sizeof(Settings.Input[index].Name)] = 0;
  String label = name;
  label.trim();
  if (label.length() == 0)
    label = "Input " + String(index + 1);
  for (byte other = 0; other < index; other++)
  {
    if (inputLabel(other) == label)
      return label + " (" + String(index + 1) + ")";
  }
  return label;
}

static String stateAsJson()
{
  JsonDocument document;
  document["power"] = appMode == APP_STANDBY_MODE ? "OFF" : "ON";
  document["volume"] = RuntimeSettings.CurrentVolume;
  document["mute"] = RuntimeSettings.Muted ? "ON" : "OFF";
  document["input"] = inputLabel(RuntimeSettings.CurrentInput);
  document["input_index"] = RuntimeSettings.CurrentInput + 1;
  String output;
  serializeJson(document, output);
  return output;
}

static void publishState(bool force)
{
  if (!client.connected())
    return;
  String state = stateAsJson();
  if (!force && state == lastPublishedState && millis() - lastStatePublish < STATE_REFRESH_INTERVAL_MS)
    return;
  if (client.publish(stateTopic.c_str(), state.c_str(), true))
  {
    lastPublishedState = state;
    lastStatePublish = millis();
  }
}

static void publishDiscoveryEntity(const char *component, const char *objectId, JsonDocument &document)
{
  document["name"] = objectId;
  document["unique_id"] = deviceId + "_" + objectId;
  document["object_id"] = deviceId + "_" + objectId;
  document["availability_topic"] = statusTopic;
  document["state_topic"] = stateTopic;
  JsonObject device = document["device"].to<JsonObject>();
  device["identifiers"][0] = deviceId;
  device["name"] = "ThePreamp";
  device["manufacturer"] = "ThePreamp";
  device["model"] = "ThePreamp Controller";
  device["sw_version"] = String(VERSION, 3);

  String topic = "homeassistant/" + String(component) + "/" + deviceId + "/" + objectId + "/config";
  String payload;
  serializeJson(document, payload);
  client.publish(topic.c_str(), payload.c_str(), true);
}

static void publishDiscovery()
{
  String base = String(config.baseTopic);
  {
    JsonDocument doc;
    doc["name"] = "Power";
    doc["command_topic"] = base + "/power/set";
    doc["value_template"] = "{{ value_json.power }}";
    doc["icon"] = "mdi:power";
    publishDiscoveryEntity("switch", "power", doc);
  }
  {
    JsonDocument doc;
    doc["name"] = "Mute";
    doc["command_topic"] = base + "/mute/set";
    doc["value_template"] = "{{ value_json.mute }}";
    doc["icon"] = "mdi:volume-off";
    publishDiscoveryEntity("switch", "mute", doc);
  }
  {
    JsonDocument doc;
    doc["name"] = "Volume";
    doc["command_topic"] = base + "/volume/set";
    doc["value_template"] = "{{ value_json.volume }}";
    doc["min"] = 0;
    doc["max"] = Settings.VolumeSteps;
    doc["step"] = 1;
    doc["mode"] = "slider";
    doc["icon"] = "mdi:volume-high";
    publishDiscoveryEntity("number", "volume", doc);
  }
  {
    JsonDocument doc;
    doc["name"] = "Input";
    doc["command_topic"] = base + "/input/set";
    doc["value_template"] = "{{ value_json.input }}";
    doc["icon"] = "mdi:audio-input-rca";
    JsonArray options = doc["options"].to<JsonArray>();
    for (byte index = 0; index < 5; index++)
    {
      if (Settings.Input[index].Active != INPUT_INACTIVATED)
        options.add(inputLabel(index));
    }
    publishDiscoveryEntity("select", "input", doc);
  }
}

static void onMessage(char *topic, byte *payload, unsigned int length)
{
  String message;
  message.reserve(length);
  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];
  message.trim();

  String command = String(topic).substring(strlen(config.baseTopic) + 1);
  bool on = message.equalsIgnoreCase("ON");
  bool off = message.equalsIgnoreCase("OFF");
  debug("MQTT: "); debug(command); debug(" = "); debugln(message);

  mil_LastUserInput = millis();

  if (command == "power/set")
  {
    if (on)
      requestedPowerCommand = KEY_ON;
    else if (off)
      requestedPowerCommand = KEY_OFF;
    return;
  }

  // Everything else only makes sense while the preamp is running normally
  if (appMode != APP_NORMAL_MODE)
    return;

  if (command == "mute/set")
  {
    if (on && !RuntimeSettings.Muted)
      mute();
    else if (off && RuntimeSettings.Muted)
      unmute();
    right_display_update();
  }
  else if (command == "volume/set")
  {
    if (message.length() > 0 && !RuntimeSettings.Muted)
      setVolume(message.toInt());
  }
  else if (command == "input/set")
  {
    for (byte index = 0; index < 5; index++)
    {
      if (Settings.Input[index].Active != INPUT_INACTIVATED && message == inputLabel(index))
      {
        setInput(index);
        break;
      }
    }
  }
  publishState(false);
}

static void disconnectClient()
{
  if (client.connected())
  {
    client.publish(statusTopic.c_str(), "offline", true);
    client.disconnect();
  }
}

static bool connectClient()
{
  String clientId = deviceId;
  bool hasCredentials = strlen(config.user) > 0;
  bool connected = client.connect(clientId.c_str(),
                                  hasCredentials ? config.user : nullptr,
                                  hasCredentials ? config.pass : nullptr,
                                  statusTopic.c_str(), 1, true, "offline");
  if (!connected)
  {
    debug("MQTT connect failed, rc="); debugln(client.state());
    return false;
  }

  client.publish(statusTopic.c_str(), "online", true);
  String base = String(config.baseTopic);
  client.subscribe((base + "/power/set").c_str());
  client.subscribe((base + "/mute/set").c_str());
  client.subscribe((base + "/volume/set").c_str());
  client.subscribe((base + "/input/set").c_str());
  publishDiscovery();
  publishState(true);
  debugln("MQTT connected");
  return true;
}

static void applyConfig()
{
  disconnectClient();
  String base = String(config.baseTopic);
  while (base.endsWith("/"))
    base.remove(base.length() - 1);
  if (base.length() == 0)
    base = "thepreamp";
  strlcpy(config.baseTopic, base.c_str(), sizeof(config.baseTopic));
  statusTopic = base + "/status";
  stateTopic = base + "/state";
  client.setServer(config.host, config.port);
  lastReconnectAttempt = 0;
}

void mqttBegin()
{
  loadConfig();
  uint64_t mac = ESP.getEfuseMac();
  char id[24];
  snprintf(id, sizeof(id), "thepreamp_%06x", (unsigned int)(mac >> 24) & 0xFFFFFF);
  deviceId = id;

  wifiClient.setTimeout(2);
  client.setBufferSize(1024);
  client.setKeepAlive(60);
  client.setCallback(onMessage);
  applyConfig();
}

void mqttLoop()
{
  if (reloadRequested)
  {
    reloadRequested = false;
    applyConfig();
  }

  if (!config.enabled || strlen(config.host) == 0 || WiFi.status() != WL_CONNECTED)
    return;

  if (!client.connected())
  {
    if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL_MS || lastReconnectAttempt == 0)
    {
      lastReconnectAttempt = millis();
      if (lastReconnectAttempt == 0)
        lastReconnectAttempt = 1;
      connectClient();
    }
    return;
  }

  client.loop();
  publishState(false);
}
