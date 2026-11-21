#include "aws_gateway_config.h"
#include <Preferences.h>
#include <LittleFS.h>

static Preferences prefs;
static const char *NS = "aws_mqtt";

void awsConfigDefaults(AwsMqttConfig &c) {
  c.wifiSsid = "";
  c.wifiPassword = "";
  c.endpoint = "";
  c.port = 8883;
  c.clientId = "NUOS_GATEWAY";
  c.thingName = "NUOS_GATEWAY";
  c.region = "ap-south-1";
  c.username = "";
  c.password = "";
  c.telemetryTopic = "nuos/gateway/{gatewayId}/telemetry";
  c.eventTopic = "nuos/gateway/{gatewayId}/event";
  c.commandTopic = "nuos/gateway/{gatewayId}/command";
  c.stateTopic = "nuos/gateway/{gatewayId}/state";
  c.availabilityTopic = "nuos/gateway/{gatewayId}/availability";
  c.qos = 1;
  c.keepAlive = 60;
  c.cleanSession = true;
  c.autoReconnect = true;
  c.lwtPayload = "offline";
  c.lwtQos = 1;
  c.lwtRetain = true;
}

bool awsConfigLoad(AwsMqttConfig &c) {
  awsConfigDefaults(c);
  if (!prefs.begin(NS, true)) return false;
  c.wifiSsid = prefs.getString("wifiSsid", c.wifiSsid);
  c.wifiPassword = prefs.getString("wifiPass", c.wifiPassword);
  c.endpoint = prefs.getString("endpoint", c.endpoint);
  c.port = prefs.getUShort("port", c.port);
  c.clientId = prefs.getString("clientId", c.clientId);
  c.thingName = prefs.getString("thingName", c.thingName);
  c.region = prefs.getString("region", c.region);
  c.username = prefs.getString("username", c.username);
  c.password = prefs.getString("password", c.password);
  c.telemetryTopic = prefs.getString("telemetry", c.telemetryTopic);
  c.eventTopic = prefs.getString("event", c.eventTopic);
  c.commandTopic = prefs.getString("command", c.commandTopic);
  c.stateTopic = prefs.getString("state", c.stateTopic);
  c.availabilityTopic = prefs.getString("availability", c.availabilityTopic);
  c.qos = prefs.getUChar("qos", c.qos);
  c.keepAlive = prefs.getUShort("keepalive", c.keepAlive);
  c.cleanSession = prefs.getBool("clean", c.cleanSession);
  c.autoReconnect = prefs.getBool("reconnect", c.autoReconnect);
  c.lwtPayload = prefs.getString("lwt", c.lwtPayload);
  c.lwtQos = prefs.getUChar("lwtqos", c.lwtQos);
  c.lwtRetain = prefs.getBool("lwtret", c.lwtRetain);
  prefs.end();
  return true;
}

bool awsConfigSave(const AwsMqttConfig &c) {
  if (!prefs.begin(NS, false)) return false;
  prefs.putString("wifiSsid", c.wifiSsid);
  prefs.putString("wifiPass", c.wifiPassword);
  prefs.putUShort("port", c.port);
  prefs.putString("clientId", c.clientId);
  prefs.putString("thingName", c.thingName);
  prefs.putString("region", c.region);
  prefs.putString("username", c.username);
  prefs.putString("password", c.password);
  prefs.putString("telemetry", c.telemetryTopic);
  prefs.putString("event", c.eventTopic);
  prefs.putString("command", c.commandTopic);
  prefs.putString("state", c.stateTopic);
  prefs.putString("availability", c.availabilityTopic);
  prefs.putUChar("qos", c.qos);
  prefs.putUShort("keepalive", c.keepAlive);
  prefs.putBool("clean", c.cleanSession);
  prefs.putBool("reconnect", c.autoReconnect);
  prefs.putString("lwt", c.lwtPayload);
  prefs.putUChar("lwtqos", c.lwtQos);
  prefs.putBool("lwtret", c.lwtRetain);
  prefs.end();
  return true;
}

static String jsonEscape(const String &s) {
  String r; r.reserve(s.length()+8);
  for (size_t i=0;i<s.length();++i) { char c=s[i]; if(c=='\\'||c=='"') r+='\\'; if(c=='\n') {r+="\\n"; continue;} r+=c; }
  return r;
}

String awsConfigJson(bool includeSecrets) {
  AwsMqttConfig c; awsConfigLoad(c);
  String j="{";
  j += "\"wifiSsid\":\""+jsonEscape(c.wifiSsid)+"\",\"wifiPassword\":\""+jsonEscape(includeSecrets?c.wifiPassword:(c.wifiPassword.length()?"********":""))+"\"";
  j += ",\"endpoint\":\""+jsonEscape(c.endpoint)+"\",\"port\":"+String(c.port);
  j += ",\"clientId\":\""+jsonEscape(c.clientId)+"\",\"thingName\":\""+jsonEscape(c.thingName)+"\",\"region\":\""+jsonEscape(c.region)+"\"";
  j += ",\"username\":\""+jsonEscape(includeSecrets?c.username:(c.username.length()?"********":""))+"\"";
  j += ",\"password\":\""+jsonEscape(includeSecrets?c.password:(c.password.length()?"********":""))+"\"";
  j += ",\"telemetryTopic\":\""+jsonEscape(c.telemetryTopic)+"\",\"eventTopic\":\""+jsonEscape(c.eventTopic)+"\",\"commandTopic\":\""+jsonEscape(c.commandTopic)+"\"";
  j += ",\"stateTopic\":\""+jsonEscape(c.stateTopic)+"\",\"availabilityTopic\":\""+jsonEscape(c.availabilityTopic)+"\"";
  j += ",\"qos\":"+String(c.qos)+",\"keepAlive\":"+String(c.keepAlive)+",\"cleanSession\":"+(c.cleanSession?"true":"false")+",\"autoReconnect\":"+(c.autoReconnect?"true":"false");
  j += ",\"lwtPayload\":\""+jsonEscape(c.lwtPayload)+"\",\"lwtQos\":"+String(c.lwtQos)+",\"lwtRetain\":"+(c.lwtRetain?"true":"false")+"}";
  return j;
}

String awsReadCertificate(const char *name) {
  if (!LittleFS.begin(true)) return "";
  File f=LittleFS.open(String("/aws/")+name, "r");
  if(!f) return "";
  String s=f.readString(); f.close(); return s;
}

bool awsSaveCertificate(const char *name, const uint8_t *data, size_t len) {
  if (!LittleFS.begin(true)) return false;
  LittleFS.mkdir("/aws");
  File f=LittleFS.open(String("/aws/")+name, "w");
  if(!f) return false;
  size_t n=f.write(data,len); f.close(); return n==len;
}

String awsCertStatusJson() {
  String j="{";
  j += "\"rootCA\":"+(awsReadCertificate("root_ca.pem").length()?String("true"):String("false"));
  j += ",\"clientCertificate\":"+(awsReadCertificate("device_cert.pem").length()?String("true"):String("false"));
  j += ",\"privateKey\":"+(awsReadCertificate("private_key.pem").length()?String("true"):String("false"));
  j += "}"; return j;
}
