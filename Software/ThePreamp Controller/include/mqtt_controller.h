#ifndef MQTT_CONTROLLER_H
#define MQTT_CONTROLLER_H

#include <Arduino.h>

struct MqttConfig
{
  bool enabled;
  char host[64];
  uint16_t port;
  char user[33];
  char pass[65];
  char baseTopic[48];
};

void mqttBegin();
void mqttLoop();
void mqttRequestReload();
const MqttConfig &mqttGetConfig();
void mqttSetConfig(const MqttConfig &config);
bool mqttIsConnected();

#endif
