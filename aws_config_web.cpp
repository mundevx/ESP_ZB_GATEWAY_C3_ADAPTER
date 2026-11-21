#include "aws_config_web.h"
#include "ble_provisioning.h"
#include "aws_gateway_config.h"
#include "web_assets.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>


static WebServer server(80);
static String uploadName;
static bool uploadOK=false;

static String zigbeeDeviceCache = "{\"devices\":[],\"count\":0}";
static String zigbeeLastEvent = "";
static String zigbeeRefreshRequestId = "";
static bool zigbeeRefreshPending = false;
static bool zigbeeRefreshCompleted = false;
static uint32_t zigbeeRefreshStartedMs = 0;
extern uint16_t sendCommand(const char *json);
extern bool isH2Online();

static String makeRequestId();

template <size_t N>
static bool jsonBody(StaticJsonDocument<N> &d)
{
  if (!server.hasArg("plain")) return false;
  return !deserializeJson(d, server.arg("plain"));
}

static String sceneKey(uint16_t groupId, uint8_t sceneId)
{
  return String("s_") + String(groupId) + String("_") + String(sceneId);
}

static bool sendH2SceneCommand(const char *command, const char *arguments)
{
  if (!command || !*command) return false;

  String requestId = makeRequestId();
  String json = String("{\"schema\":\"nuos.gateway.v2\",") +
                String("\"type\":\"command\",") +
                String("\"requestId\":\"") + requestId + String("\",") +
                String("\"command\":\"") + command + String("\",") +
                String("\"arguments\":") +
                String(arguments && *arguments ? arguments : "{}") + String("}");

  return sendCommand(json.c_str()) != 0;
}


/* -------------------------------------------------------------------------- */
/* Canonical NUOS scene service                                                */
/* -------------------------------------------------------------------------- */
static Preferences scenePrefs;
static bool scenePrefsOpen = false;

static void sceneStoreInit()
{
  if (!scenePrefsOpen) {
    scenePrefsOpen = scenePrefs.begin("nuos_scene", false);
  }
}

static String sceneStoreRead()
{
  sceneStoreInit();
  if (!scenePrefsOpen) return String("{\"scenes\":[]}");

  JsonDocument index;
  String idx = scenePrefs.getString("index", "[]");
  if (deserializeJson(index, idx)) return String("{\"scenes\":[]}");
  JsonArray keys = index.as<JsonArray>();

  JsonDocument out;
  JsonArray scenes = out["scenes"].to<JsonArray>();
  for (JsonVariant kv : keys) {
    const char *key = kv.as<const char*>();
    if (!key || !*key) continue;
    String raw = scenePrefs.getString(key, "");
    if (!raw.length()) continue;
    JsonDocument one;
    if (!deserializeJson(one, raw)) scenes.add(one.as<JsonObject>());
  }
  String result;
  serializeJson(out, result);
  return result;
}

static bool sceneStoreWrite(const String &json)
{
  sceneStoreInit();
  if (!scenePrefsOpen) return false;

  JsonDocument doc;
  if (deserializeJson(doc, json)) return false;
  JsonArray scenes = doc["scenes"].as<JsonArray>();

  /* Remove old scene keys from the index and rebuild them. */
  JsonDocument oldIndex;
  String oldIdx = scenePrefs.getString("index", "[]");
  deserializeJson(oldIndex, oldIdx);
  JsonArray oldKeys = oldIndex.as<JsonArray>();
  for (JsonVariant kv : oldKeys) {
    const char *key = kv.as<const char*>();
    if (key && *key) scenePrefs.remove(key);
  }

  JsonDocument newIndex;
  JsonArray keys = newIndex.to<JsonArray>();
  bool ok = true;
  for (JsonObject scene : scenes) {
    uint16_t gid = scene["groupId"] | 0;
    uint8_t sid = (uint8_t)(scene["sceneId"] | 0);
    String key = String("s_") + String(gid) + String("_") + String(sid);
    String raw;
    serializeJson(scene, raw);
    if (raw.length() > 3000 || scenePrefs.putString(key.c_str(), raw) == 0) ok = false;
    keys.add(key);
  }
  String indexJson;
  serializeJson(newIndex, indexJson);
  if (scenePrefs.putString("index", indexJson) == 0) ok = false;
  return ok;
}

static bool sceneFindDeviceEndpoint(JsonDocument &devicesDoc,
                                    const char *deviceId,
                                    uint8_t endpointId,
                                    uint16_t &shortAddress)
{
  JsonArray devices = devicesDoc["devices"].as<JsonArray>();
  if (devices.isNull()) return false;
  for (JsonObject d : devices) {
    const char *id = d["deviceId"] | "";
    if (strcmp(id, deviceId) != 0) continue;
    JsonArray eps = d["endpoints"].as<JsonArray>();
    if (eps.isNull()) return false;
    for (JsonObject ep : eps) {
      if ((uint8_t)(ep["endpointNumber"] | 0) == endpointId) {
        JsonVariant sv = d["zigbee"]["shortAddress"];
        if (sv.is<const char*>()) {
          const char *x = sv.as<const char*>();
          shortAddress = (uint16_t)strtoul(x, nullptr, 0);
        } else {
          shortAddress = (uint16_t)(sv | 0);
        }
        return shortAddress != 0;
      }
    }
  }
  return false;
}

static bool scenePersistReplace(const JsonObject &scene)
{
  JsonDocument store;
  if (deserializeJson(store, sceneStoreRead())) return false;
  JsonArray scenes = store["scenes"].as<JsonArray>();
  if (scenes.isNull()) scenes = store.createNestedArray("scenes");

  uint16_t gid = scene["groupId"] | 0;
  uint8_t sid = (uint8_t)(scene["sceneId"] | 0);
  String key = sceneKey(gid, sid);
  for (JsonObject existing : scenes) {
    uint16_t eg = existing["groupId"] | 0;
    uint8_t es = (uint8_t)(existing["sceneId"] | 0);
    if (sceneKey(eg, es) == key) {
      existing.set(scene);
      String out; serializeJson(store, out);
      return sceneStoreWrite(out);
    }
  }
  JsonObject added = scenes.createNestedObject();
  added.set(scene);
  String out; serializeJson(store, out);
  return sceneStoreWrite(out);
}

static bool scenePersistDelete(uint16_t groupId, uint8_t sceneId)
{
  JsonDocument store;
  if (deserializeJson(store, sceneStoreRead())) return false;
  JsonArray scenes = store["scenes"].as<JsonArray>();
  if (scenes.isNull()) return true;
  for (size_t i = scenes.size(); i > 0; --i) {
    JsonObject s = scenes[i - 1].as<JsonObject>();
    if ((uint16_t)(s["groupId"] | 0) == groupId &&
        (uint8_t)(s["sceneId"] | 0) == sceneId) {
      scenes.remove(i - 1);
    }
  }
  String out; serializeJson(store, out);
  return sceneStoreWrite(out);
}

static void registerSceneApi()
{
  sceneStoreInit();

  server.on("/api/scenes", HTTP_GET, [](){
    server.send(200, "application/json", sceneStoreRead());
  });

  server.on("/api/scene/create", HTTP_POST, [](){
    StaticJsonDocument<4096> d;
    if (!jsonBody(d) || !d["scene"].is<JsonObject>()) {
      server.send(400, "application/json", "{\"error\":\"scene object required\"}");
      return;
    }

    JsonObject scene = d["scene"].as<JsonObject>();
    uint16_t groupId = scene["groupId"] | 0;
    uint8_t sceneId = (uint8_t)(scene["sceneId"] | 0);
    JsonArray members = scene["members"].as<JsonArray>();
    if (groupId == 0 || sceneId == 0 || members.isNull() || members.size() == 0) {
      server.send(400, "application/json", "{\"error\":\"groupId, sceneId and at least one member are required\"}");
      return;
    }

    /* Canonical configuration is persisted before Zigbee programming. */
    scene["schema"] = "nuos.canonical.scene.v1";
    scene["status"] = "configuring";
    scene["updatedAtMs"] = (uint32_t)millis();
    if (!scenePersistReplace(scene)) {
      server.send(500, "application/json", "{\"error\":\"scene persistence failed\"}");
      return;
    }

    JsonDocument cache;
    deserializeJson(cache, zigbeeDeviceCache);
    uint16_t queuedMembers = 0;
    uint16_t rejectedMembers = 0;

    for (JsonObject m : members) {
      const char *deviceId = m["deviceId"] | "";
      uint8_t ep = (uint8_t)(m["endpointId"] | m["endpointNumber"] | 0);
      uint16_t shortAddr = 0xFFFF;
      if (!deviceId[0] || ep == 0 || !sceneFindDeviceEndpoint(cache, deviceId, ep, shortAddr)) {
        rejectedMembers++;
        continue;
      }

      String args = String("{\"shortAddress\":") + String(shortAddr) +
                    String(",\"endpointId\":") + String(ep) +
                    String(",\"groupId\":") + String(groupId) +
                    String(",\"sceneId\":") + String(sceneId) + String("}");
      if (sendH2SceneCommand("scene.member.configure", args.c_str())) queuedMembers++;
      else rejectedMembers++;
    }

    JsonObject controller = scene["controller"].as<JsonObject>();
    bool controllerQueued = false;
    if (!controller.isNull()) {
      const char *deviceId = controller["deviceId"] | "";
      uint8_t ep = (uint8_t)(controller["endpointId"] | controller["endpointNumber"] | 0);
      uint16_t shortAddr = 0xFFFF;
      if (deviceId[0] && ep && sceneFindDeviceEndpoint(cache, deviceId, ep, shortAddr)) {
        String args = String("{\"shortAddress\":") + String(shortAddr) +
                      String(",\"endpointId\":") + String(ep) +
                      String(",\"groupId\":") + String(groupId) +
                      String(",\"sceneId\":") + String(sceneId) + String("}");
        controllerQueued = sendH2SceneCommand("scene.controller.configure", args.c_str());
      }
    }

    StaticJsonDocument<1024> result;
    result["status"] = "accepted";
    result["sceneId"] = sceneId;
    result["groupId"] = groupId;
    result["queuedMembers"] = queuedMembers;
    result["rejectedMembers"] = rejectedMembers;
    result["controllerQueued"] = controllerQueued;
    String out; serializeJson(result, out);
    server.send(202, "application/json", out);
  });

  server.on("/api/scene/recall", HTTP_POST, [](){
    StaticJsonDocument<2048> d;
    if (!jsonBody(d)) { server.send(400,"application/json","{\"error\":\"invalid JSON\"}"); return; }
    uint16_t gid = d["groupId"] | d["group_id"] | 0;
    uint8_t sid = (uint8_t)(d["sceneId"] | d["scene_id"] | 0);
    if (!gid || !sid) { server.send(400,"application/json","{\"error\":\"groupId and sceneId required\"}"); return; }
    String args = String("{\"groupId\":") + String(gid) + String(",\"sceneId\":") + String(sid) + String("}");
    bool queued = sendH2SceneCommand("scene.recall", args.c_str());
    server.send(queued ? 202 : 503, "application/json", queued ? "{\"status\":\"accepted\"}" : "{\"error\":\"H2 command queue failed\"}");
  });

  server.on("/api/scene/delete", HTTP_POST, [](){
    StaticJsonDocument<2048> d;
    if (!jsonBody(d)) { server.send(400,"application/json","{\"error\":\"invalid JSON\"}"); return; }
    uint16_t gid = d["groupId"] | 0;
    uint8_t sid = (uint8_t)(d["sceneId"] | 0);
    if (!gid || !sid) { server.send(400,"application/json","{\"error\":\"groupId and sceneId required\"}"); return; }

    JsonDocument store;
    deserializeJson(store, sceneStoreRead());
    JsonArray scenes = store["scenes"].as<JsonArray>();
    JsonObject target;
    for (JsonObject s : scenes) {
      if ((uint16_t)(s["groupId"] | 0) == gid && (uint8_t)(s["sceneId"] | 0) == sid) { target=s; break; }
    }
    JsonDocument cache; deserializeJson(cache, zigbeeDeviceCache);
    if (!target.isNull()) {
      JsonArray members = target["members"].as<JsonArray>();
      for (JsonObject m : members) {
        const char *deviceId=m["deviceId"]|""; uint8_t ep=(uint8_t)(m["endpointId"]|0); uint16_t sa=0xFFFF;
        if (!deviceId[0] || !ep || !sceneFindDeviceEndpoint(cache,deviceId,ep,sa)) continue;
        String args=String("{\"shortAddress\":")+String(sa)+String(",\"endpointId\":")+String(ep)+String(",\"groupId\":")+String(gid)+String(",\"sceneId\":")+String(sid)+String("}");
        sendH2SceneCommand("scene.member.remove",args.c_str());
      }
    }
    bool ok=scenePersistDelete(gid,sid);
    server.send(ok?202:500,"application/json",ok?"{\"status\":\"accepted\"}":"{\"error\":\"scene persistence failed\"}");
  });
}

static String makeRequestId(){ static uint32_t n=1; return String("web_")+String(millis())+String("_")+String(n++); }

// Called by the C3 UART dispatcher whenever H2 sends a complete JSON message.
void zigbeeUiHandleH2Message(uint8_t type, const char *json){
  if(!json) return;
  StaticJsonDocument<16384> d;
  if(deserializeJson(d,json)) return;
  const char *et=d["eventType"]|"";
  if(type==3 && d["data"]["devices"].is<JsonArray>()){
    String out; serializeJson(d["data"],out); zigbeeDeviceCache=out;

    const char *rid = d["requestId"] | "";
    if (zigbeeRefreshPending && zigbeeRefreshRequestId == String(rid)) {
      zigbeeRefreshPending = false;
      zigbeeRefreshCompleted = true;
      Serial.printf("[ZIGBEE UI] Physical H2 device refresh completed: %u devices\n",
                    (unsigned)(d["data"]["count"] | 0));
    }
  } else if(type==4 && !strcmp(et,"scene.triggered")){
    /* Tuya 12-scene normalization: 4 buttons x (single/double/long).
     * sceneIndex = (buttonEndpoint - 1) * 3 + action + 1.
     * The canonical scene configuration maps this trigger to sceneId/groupId.
     */
    const char *did = d["deviceId"] | "";
    uint8_t ep = (uint8_t)(d["endpointId"] | d["sourceEndpoint"] | 0);
    uint8_t sceneIndex = (uint8_t)(d["sceneIndex"] | 0);
    if(did[0] && ep && sceneIndex){
      JsonDocument store;
      if(!deserializeJson(store,sceneStoreRead())){
        JsonArray arr=store["scenes"].as<JsonArray>();
        for(JsonObject sc:arr){
          JsonObject c=sc["controller"].as<JsonObject>();
          if(c.isNull()) continue;
          if(String((const char*)(c["deviceId"]|""))==did &&
             (uint8_t)(c["endpointId"]|0)==ep &&
             (((uint8_t)(c["sceneIndex"]|0)==sceneIndex) ||
              String((const char*)(c["triggerType"]|""))==String((const char*)(d["triggerType"]|"")))){
            uint16_t gid=sc["groupId"]|0; uint8_t sid=(uint8_t)(sc["sceneId"]|0);
            String args=String("{\"groupId\":")+String(gid)+String(",\"sceneId\":")+String(sid)+String("}");
            sendH2SceneCommand("scene.recall",args.c_str());
            break;
          }
        }
      }
    }
  } else if(type==4){
    zigbeeLastEvent=String(json);
    JsonObject dev=d["device"].as<JsonObject>();
    const char *id=dev["deviceId"]|d["deviceId"]|"";
    if(!*id) return;
    StaticJsonDocument<16384> c; if(deserializeJson(c,zigbeeDeviceCache)) return;
    JsonArray a=c["devices"].as<JsonArray>(); if(a.isNull()) a=c.createNestedArray("devices");
    int idx=-1; int i=0; for(JsonObject x:a){if(String((const char*)(x["deviceId"]|""))==id){idx=i;break;} i++;}
    if(!strcmp(et,"device.removed")){ if(idx>=0) a.remove(idx); }
    else if(idx>=0){ JsonObject x=a[idx]; if(dev) x.set(dev); if(!strcmp(et,"connectivity.changed")) x["connectivity"]["state"]=d["connectivity"]["state"]|"offline"; }
    else if(dev){ a.add(dev); }
    c["count"]=a.size(); String out; serializeJson(c,out); zigbeeDeviceCache=out;
  }
}

static void sendJson(const String &s){server.send(200,"application/json",s);}
static AwsMqttConfig cfgFromJson(JsonDocument &d){
  AwsMqttConfig c; awsConfigLoad(c);
  if(d.containsKey("wifiSsid")) c.wifiSsid=d["wifiSsid"].as<String>();
  if(d.containsKey("wifiPassword")) c.wifiPassword=d["wifiPassword"].as<String>();
  if(d.containsKey("endpoint")) c.endpoint=d["endpoint"].as<String>();
  if(d.containsKey("clientId")) c.clientId=d["clientId"].as<String>();
  if(d.containsKey("thingName")) c.thingName=d["thingName"].as<String>();
  if(d.containsKey("region")) c.region=d["region"].as<String>();
  if(d.containsKey("username")) c.username=d["username"].as<String>();
  if(d.containsKey("password")) c.password=d["password"].as<String>();
  if(d.containsKey("telemetryTopic")) c.telemetryTopic=d["telemetryTopic"].as<String>();
  if(d.containsKey("eventTopic")) c.eventTopic=d["eventTopic"].as<String>();
  if(d.containsKey("commandTopic")) c.commandTopic=d["commandTopic"].as<String>();
  if(d.containsKey("stateTopic")) c.stateTopic=d["stateTopic"].as<String>();
  if(d.containsKey("availabilityTopic")) c.availabilityTopic=d["availabilityTopic"].as<String>();
  if(d.containsKey("lwtPayload")) c.lwtPayload=d["lwtPayload"].as<String>();
  if(d.containsKey("port")) c.port=d["port"].as<uint16_t>();
  if(d.containsKey("qos")) c.qos=d["qos"].as<uint8_t>();
  if(d.containsKey("keepAlive")) c.keepAlive=d["keepAlive"].as<uint16_t>();
  if(d.containsKey("lwtQos")) c.lwtQos=d["lwtQos"].as<uint8_t>();
  if(d.containsKey("cleanSession")) c.cleanSession=d["cleanSession"].as<bool>();
  if(d.containsKey("autoReconnect")) c.autoReconnect=d["autoReconnect"].as<bool>();
  if(d.containsKey("lwtRetain")) c.lwtRetain=d["lwtRetain"].as<bool>();
  return c;
}
void awsWebBegin(){
 // Web UI is compiled directly into the C3 application flash.
 // No LittleFS mount or filesystem upload is required.
 server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", index_html); });
 server.on("/index.html", HTTP_GET, [](){ server.send_P(200, "text/html", index_html); });
 server.on("/style.css", HTTP_GET, [](){ server.send_P(200, "text/css", style_css); });
 server.on("/app.js", HTTP_GET, [](){ server.send_P(200, "application/javascript", app_js); });
 server.on("/api/ble/status",HTTP_GET,[]{
   String out=String("{\"initialized\":")+(nuosBleProvisioningActive()||nuosBleProvisioningConnected()?"true":"false")+String(",\"advertising\":")+(nuosBleProvisioningActive()?"true":"false")+String(",\"connected\":")+(nuosBleProvisioningConnected()?"true":"false")+String("}");
   sendJson(out);
 });
 server.on("/api/ble/start",HTTP_POST,[]{ nuosBleProvisioningStart(); server.send(200,"application/json","{\"status\":\"started\"}"); });
 server.on("/api/ble/stop",HTTP_POST,[]{ nuosBleProvisioningStop(); server.send(200,"application/json","{\"status\":\"stopped\"}"); });
 server.on("/api/status",HTTP_GET,[]{
   bool wifi = WiFi.status()==WL_CONNECTED;
   bool h2 = isH2Online();
   String out=String("{\"h2\":")+(h2?"true":"false")+String(",\"wifi\":")+(wifi?"true":"false")+String(",\"awsConnected\":false,\"freeHeap\":")+String(ESP.getFreeHeap())+String("}");
   sendJson(out);
 });
 server.on("/api/network",HTTP_GET,[]{
   String out=String("{\"wifiStatus\":")+String(WiFi.status())+String(",\"ssid\":\"")+String(WiFi.SSID())+String("\",\"ip\":\"")+WiFi.localIP().toString()+String("\",\"apActive\":")+(WiFi.getMode()==WIFI_AP||WiFi.getMode()==WIFI_AP_STA?"true":"false")+String(",\"apSsid\":\"NUOS-ZB-GATEWAY\",\"apIp\":\"")+WiFi.softAPIP().toString()+String("\",\"gateway\":\"")+WiFi.gatewayIP().toString()+String("\",\"rssi\":")+String(WiFi.RSSI())+String("}"); sendJson(out);
 });
 server.on("/api/system",HTTP_GET,[]{
   String out=String("{\"chip\":\"ESP32-C3\",\"freeHeap\":")+String(ESP.getFreeHeap())+String(",\"uptimeMs\":")+String(millis())+String(",\"flashSize\":")+String(ESP.getFlashChipSize())+String("}"); sendJson(out);
 });
 server.on("/api/aws/config",HTTP_GET,[]{sendJson(awsConfigJson(false));});
 server.on("/api/aws/certs",HTTP_GET,[]{sendJson(awsCertStatusJson());});
 server.on("/api/aws/config",HTTP_POST,[]{if(!server.hasArg("plain")){server.send(400,"text/plain","missing JSON");return;}StaticJsonDocument<4096>d; if(deserializeJson(d,server.arg("plain"))){server.send(400,"text/plain","invalid JSON");return;}AwsMqttConfig c=cfgFromJson(d);bool ok=awsConfigSave(c);server.send(ok?200:500,"text/plain",ok?"saved":"save failed");});
 server.on("/api/aws/test",HTTP_POST,[]{server.send(200,"application/json","{\"status\":\"test_requested\",\"note\":\"MQTT test is executed by the C3 MQTT task after configuration reload.\"}");});
 // Zigbee Bridge UI/API -- NUOS Ecosystem V2 public surface
 server.on("/api/zigbee/devices",HTTP_GET,[]{
   server.send(200,"application/json",zigbeeDeviceCache);
 });
 server.on("/api/zigbee/refresh",HTTP_POST,[]{
   String id=makeRequestId();
   String j=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+
            id+String("\",\"command\":\"device.list\",\"arguments\":{}}");
   zigbeeRefreshRequestId=id;
   zigbeeRefreshPending=true;
   zigbeeRefreshCompleted=false;
   zigbeeRefreshStartedMs=millis();
   bool queued=sendCommand(j.c_str());
   if(!queued){
     zigbeeRefreshPending=false;
     zigbeeRefreshCompleted=false;
     server.send(503,"application/json","{\"error\":\"H2 UART command could not be queued\"}");
     return;
   }
   server.send(202,"application/json",String("{\"status\":\"accepted\",\"requestId\":\"")+id+String("\"}"));
 });
 server.on("/api/zigbee/refresh/status",HTTP_GET,[]{
   String id=server.arg("requestId");
   bool match=(id.length()==0 || id==zigbeeRefreshRequestId);
   bool ready=match && zigbeeRefreshCompleted && zigbeeRefreshRequestId.length();
   bool timeout=match && zigbeeRefreshPending && (millis()-zigbeeRefreshStartedMs>5000);
   if(timeout){
     zigbeeRefreshPending=false;
     zigbeeRefreshCompleted=false;
     ready=false;
   }
   String out=String("{\"ready\":")+(ready?"true":"false")+
              String(",\"pending\":")+(zigbeeRefreshPending?"true":"false")+
              String(",\"timeout\":")+(timeout?"true":"false")+
              String(",\"requestId\":\"")+zigbeeRefreshRequestId+String("\",\"devices\":")+zigbeeDeviceCache+String("}");
   server.send(timeout?504:200,"application/json",out);
 });
 server.on("/api/zigbee/commissioning/start",HTTP_POST,[]{StaticJsonDocument<1024>d;uint16_t sec=180;if(jsonBody(d)&&d.containsKey("duration"))sec=(uint16_t)d["duration"].as<int>();if(sec<1)sec=1;if(sec>180)sec=180;String id=makeRequestId();String j=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+id+String("\",\"command\":\"commissioning.start\",\"arguments\":{\"duration\":")+String(sec)+String("}}");sendCommand(j.c_str());server.send(200,"text/plain","commissioning.start requested");});
 server.on("/api/zigbee/commissioning/stop",HTTP_POST,[]{String id=makeRequestId();String j=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+id+String("\",\"command\":\"commissioning.stop\",\"arguments\":{}}");sendCommand(j.c_str());server.send(200,"text/plain","commissioning.stop requested");});
 server.on("/api/zigbee/remove",HTTP_POST,[]{StaticJsonDocument<1024>d;if(!jsonBody(d)||!d["deviceId"].is<const char*>()){server.send(400,"text/plain","deviceId required");return;}String id=makeRequestId();String j=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+id+String("\",\"command\":\"device.remove\",\"deviceId\":\"")+String(d["deviceId"].as<const char*>())+String("\",\"arguments\":{}}");sendCommand(j.c_str());server.send(200,"text/plain","remove requested");});
 server.on("/api/zigbee/control",HTTP_POST,[]{
   StaticJsonDocument<2048>d;
   if(!jsonBody(d)){server.send(400,"application/json","{\"error\":\"invalid JSON\"}");return;}

   /* Accept both the compact local REST body and a complete NUOS V2 request.
    * Compact: {deviceId,endpoint,set:{...}}
    * V2:      {schema,type,requestId,command,arguments:{deviceId,endpoint,set:{...}}}
    */
   JsonObject args=d["arguments"].as<JsonObject>();
   const char *deviceId=d["deviceId"].as<const char*>();
   if((deviceId==nullptr || !*deviceId) && !args.isNull())
      deviceId=args["deviceId"].as<const char*>();
   if(deviceId==nullptr || !*deviceId){server.send(400,"text/plain","deviceId required");return;}

   int ep=1;
   if(d.containsKey("endpoint")) ep=d["endpoint"].as<int>();
   else if(d.containsKey("endpointId")) ep=d["endpointId"].as<int>();
   else if(!args.isNull() && args.containsKey("endpoint")) ep=args["endpoint"].as<int>();
   else if(!args.isNull() && args.containsKey("endpointId")) ep=args["endpointId"].as<int>();

   JsonObject setv=d["set"].as<JsonObject>();
   if(setv.isNull() && !args.isNull()) setv=args["set"].as<JsonObject>();
   if(setv.isNull()){server.send(400,"application/json","{\"error\":\"set object required\"}");return;}

   String setJson;serializeJson(setv,setJson);
   String id=makeRequestId();
   String full=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+id+String("\",\"command\":\"status.set\",\"deviceId\":\"")+String(deviceId)+String("\",\"endpointId\":")+String(ep)+String(",\"arguments\":")+setJson+String("}");
   sendCommand(full.c_str());
   server.send(200,"text/plain","control requested");
 });
 server.on("/api/zigbee/configure",HTTP_POST,[]{StaticJsonDocument<2048>d;if(!jsonBody(d)||!d["deviceId"].is<const char*>()){server.send(400,"text/plain","deviceId required");return;}String id=makeRequestId();String args;serializeJson(d["arguments"],args);String full=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+id+String("\",\"command\":\"device.configure\",\"deviceId\":\"")+String(d["deviceId"].as<const char*>())+String("\",\"arguments\":")+args+String("}");sendCommand(full.c_str());server.send(200,"text/plain","configuration requested");});
 server.on("/api/zigbee/device",HTTP_GET,[]{String id=server.arg("deviceId");if(!id.length()){server.send(400,"application/json","{\"error\":\"deviceId required\"}");return;}StaticJsonDocument<16384>d;if(!deserializeJson(d,zigbeeDeviceCache)){for(JsonObject x:d["devices"].as<JsonArray>()){if(String((const char*)(x["deviceId"]|""))==id){String out;serializeJson(x,out);server.send(200,"application/json",out);return;}}}String cid=makeRequestId();String j=String("{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"")+cid+String("\",\"command\":\"device.get\",\"deviceId\":\"")+id+String("\",\"arguments\":{}}");sendCommand(j.c_str());server.send(202,"application/json",String("{\"status\":\"requested\",\"deviceId\":\"")+id+String("\"}"));});
 registerSceneApi();
 server.on("/api/aws/cert",HTTP_POST,[]{server.send(uploadOK?200:500,"text/plain",uploadOK?"certificate uploaded":"certificate upload failed");});
 server.onFileUpload([](){HTTPUpload &u=server.upload();if(u.status==UPLOAD_FILE_START){uploadName=server.arg("name");uploadOK=false;}else if(u.status==UPLOAD_FILE_WRITE){uploadOK=awsSaveCertificate(uploadName.c_str(),u.buf,u.currentSize);}else if(u.status==UPLOAD_FILE_END){}});
 server.begin();
}
void awsWebLoop(){server.handleClient();}
