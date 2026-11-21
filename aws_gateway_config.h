#pragma once
#include <Arduino.h>

struct AwsMqttConfig {
  String wifiSsid;
  String wifiPassword;
  String endpoint;
  uint16_t port;
  String clientId;
  String thingName;
  String region;
  String username;
  String password;
  String telemetryTopic;
  String eventTopic;
  String commandTopic;
  String stateTopic;
  String availabilityTopic;
  uint8_t qos;
  uint16_t keepAlive;
  bool cleanSession;
  bool autoReconnect;
  String lwtPayload;
  uint8_t lwtQos;
  bool lwtRetain;
};

void awsConfigDefaults(AwsMqttConfig &cfg);
bool awsConfigLoad(AwsMqttConfig &cfg);
bool awsConfigSave(const AwsMqttConfig &cfg);
String awsConfigJson(bool includeSecrets = false);
String awsCertStatusJson();
bool awsSaveCertificate(const char *name, const uint8_t *data, size_t len);
String awsReadCertificate(const char *name);
