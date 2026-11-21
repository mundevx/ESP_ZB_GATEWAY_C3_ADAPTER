#include "ble_provisioning.h"
#include "aws_gateway_config.h"
#include "aws_mqtt_gateway.h"

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// NUOS Gateway BLE provisioning GATT service.
// App -> C3: write JSON to COMMAND characteristic.
// C3 -> App: notifications on RESPONSE and STATUS characteristics.
static const char *BLE_DEVICE_NAME = "NUOS-GW";
static const char *SERVICE_UUID = "7B6E0001-6D75-4F55-9B4E-4E554F534757";
static const char *INFO_UUID    = "7B6E0002-6D75-4F55-9B4E-4E554F534757";
static const char *COMMAND_UUID = "7B6E0003-6D75-4F55-9B4E-4E554F534757";
static const char *RESPONSE_UUID= "7B6E0004-6D75-4F55-9B4E-4E554F534757";
static const char *STATUS_UUID  = "7B6E0005-6D75-4F55-9B4E-4E554F534757";

static BLEServer *server = nullptr;
static BLECharacteristic *infoChar = nullptr;
static BLECharacteristic *commandChar = nullptr;
static BLECharacteristic *responseChar = nullptr;
static BLECharacteristic *statusChar = nullptr;
static bool connected = false;
// BLE lifecycle is controlled by the web UI.
// bleInitialized: GATT server has been created once.
// bleActive: advertising is currently enabled by the web UI.
static bool bleInitialized = false;
static bool bleActive = false;
static bool pendingWifi = false;
static String pendingSsid;
static String pendingPassword;
static uint32_t lastStatusMs = 0;

static String jsonEscape(const String &s) {
  String r; r.reserve(s.length() + 8);
  for (size_t i=0; i<s.length(); ++i) {
    char c=s[i];
    if (c=='\\' || c=='"') r+='\\';
    if (c=='\n') { r += "\\n"; continue; }
    if (c=='\r') { r += "\\r"; continue; }
    r += c;
  }
  return r;
}

static String gatewayIdForBle() {
  AwsMqttConfig c;
  awsConfigLoad(c);
  if (c.thingName.length()) return c.thingName;
  uint64_t mac = ESP.getEfuseMac();
  char id[32];
  snprintf(id, sizeof(id), "NUOS-GW-%06lX", (unsigned long)(mac & 0xFFFFFFUL));
  return String(id);
}

static String macString() {
  uint64_t mac = ESP.getEfuseMac();
  char b[18];
  snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X",
           (unsigned)((mac >> 40) & 0xFF), (unsigned)((mac >> 32) & 0xFF),
           (unsigned)((mac >> 24) & 0xFF), (unsigned)((mac >> 16) & 0xFF),
           (unsigned)((mac >> 8) & 0xFF), (unsigned)(mac & 0xFF));
  return String(b);
}

static String gatewayInfoJson() {
  AwsMqttConfig c;
  awsConfigLoad(c);
  String j="{";
  j += "\"schema\":\"nuos.gateway.v2\"";
  j += ",\"type\":\"gateway.info\"";
  j += ",\"gatewayId\":\""+jsonEscape(gatewayIdForBle())+"\"";
  j += ",\"productId\":\"NUOS_ZIGBEE_GATEWAY\"";
  j += ",\"deviceType\":\"zigbee_gateway\"";
  j += ",\"mac\":\""+macString()+"\"";
  j += ",\"chip\":\"ESP32-C3\"";
  j += ",\"firmware\":\"1.0.0\"";
  j += ",\"protocolVersion\":\"1.0\"";
  j += ",\"bleProvisioning\":true";
  j += ",\"wifiConfigured\":"+(c.wifiSsid.length()?String("true"):String("false"));
  j += ",\"wifiConnected\":"+(WiFi.status()==WL_CONNECTED?String("true"):String("false"));
  j += ",\"mqttConnected\":"+(awsMqttIsConnected()?String("true"):String("false"));
  j += "}";
  return j;
}

static void notify(BLECharacteristic *ch, const String &payload) {
  if (!ch || !connected) return;
  ch->setValue(payload.c_str());
  ch->notify();
}

static void sendResponse(const char *requestId, bool ok, const char *message) {
  String j="{";
  j += "\"schema\":\"nuos.gateway.v2\",\"type\":\"response\"";
  if(requestId && *requestId) j += ",\"requestId\":\""+jsonEscape(requestId)+"\"";
  j += ",\"status\":\""+(ok?String("ok"):String("failed"))+"\"";
  j += ",\"message\":\""+jsonEscape(message?String(message):String(""))+"\"}";
  notify(responseChar,j);
}

class NuosServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    (void)s;
    connected=true;
    // Once the app has found and connected to the gateway, stop advertising.
    // Keep the existing GATT connection alive so provisioning can continue.
    bleActive=false;
    BLEDevice::getAdvertising()->stop();
    Serial.println("[BLE] APP CONNECTED - advertising stopped");
    notify(responseChar, gatewayInfoJson());
    notify(statusChar, gatewayInfoJson());
  }
  void onDisconnect(BLEServer *s) override {
    (void)s;
    connected=false;
    Serial.println("[BLE] APP DISCONNECTED");
    // Do not restart automatically. BLE provisioning is controlled only from the web UI.
    if(bleActive) BLEDevice::startAdvertising();
  }
};

class NuosCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    String input = ch->getValue();
    if(input.length() == 0) return;
    Serial.printf("[BLE] COMMAND RX: %s\n", input.c_str());

    StaticJsonDocument<2048> d;
    DeserializationError err=deserializeJson(d,input);
    if(err) {
      sendResponse("",false,"INVALID_JSON");
      return;
    }

    const char *cmd=d["cmd"] | d["command"] | "";
    const char *requestId=d["requestId"] | "";

    if(!strcmp(cmd,"gateway.info")) {
      notify(responseChar,gatewayInfoJson());
      return;
    }

    if(!strcmp(cmd,"wifi.scan")) {
      int n=WiFi.scanNetworks(false,true);
      StaticJsonDocument<2048> out;
      out["schema"]="nuos.gateway.v2"; out["type"]="wifi.scan.result";
      out["requestId"]=requestId;
      JsonArray a=out.createNestedArray("networks");
      for(int i=0;i<n && i<30;i++) {
        JsonObject o=a.createNestedObject();
        o["ssid"]=WiFi.SSID(i); o["rssi"]=WiFi.RSSI(i); o["channel"]=WiFi.channel(i);
      }
      String payload; serializeJson(out,payload); notify(responseChar,payload); WiFi.scanDelete();
      return;
    }

    if(!strcmp(cmd,"wifi.configure")) {
      const char *ssid=d["ssid"] | "";
      const char *password=d["password"] | "";
      if(!ssid[0]) { sendResponse(requestId,false,"SSID_REQUIRED"); return; }
      if(strlen(password)<8) { sendResponse(requestId,false,"PASSWORD_MIN_8_CHARACTERS"); return; }
      pendingSsid=ssid; pendingPassword=password; pendingWifi=true;
      sendResponse(requestId,true,"WIFI_CONFIGURATION_ACCEPTED");
      return;
    }

    if(!strcmp(cmd,"wifi.status")) {
      StaticJsonDocument<1024> out;
      out["schema"]="nuos.gateway.v2"; out["type"]="wifi.status"; out["requestId"]=requestId;
      out["connected"]=(WiFi.status()==WL_CONNECTED); out["ssid"]=WiFi.SSID();
      out["ip"]=WiFi.localIP().toString(); out["rssi"]=WiFi.RSSI();
      String payload; serializeJson(out,payload); notify(responseChar,payload); return;
    }

    if(!strcmp(cmd,"cloud.status")) {
      sendResponse(requestId,awsMqttIsConnected(),awsMqttIsConnected()?"MQTT_CONNECTED":"MQTT_DISCONNECTED");
      return;
    }

    if(!strcmp(cmd,"provisioning.status")) {
      notify(responseChar,gatewayInfoJson()); return;
    }

    sendResponse(requestId,false,"UNKNOWN_COMMAND");
  }
};

void nuosBleProvisioningBegin() {
  if(bleInitialized) return;
  BLEDevice::init(BLE_DEVICE_NAME);
  server=BLEDevice::createServer();
  server->setCallbacks(new NuosServerCallbacks());
  BLEService *service=server->createService(SERVICE_UUID);

  infoChar=service->createCharacteristic(INFO_UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  infoChar->addDescriptor(new BLE2902());
  commandChar=service->createCharacteristic(COMMAND_UUID,BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_WRITE_NR);
  responseChar=service->createCharacteristic(RESPONSE_UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  responseChar->addDescriptor(new BLE2902());
  statusChar=service->createCharacteristic(STATUS_UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  statusChar->addDescriptor(new BLE2902());
  commandChar->setCallbacks(new NuosCommandCallbacks());

  String info=gatewayInfoJson();
  infoChar->setValue(info.c_str());
  responseChar->setValue(info.c_str());
  statusChar->setValue(info.c_str());
  service->start();

  BLEAdvertising *adv=BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  bleInitialized=true;
  bleActive=false;
  Serial.printf("[BLE] Provisioning initialized (advertising OFF): %s | Gateway ID=%s\n",BLE_DEVICE_NAME,gatewayIdForBle().c_str());
}

void nuosBleProvisioningStart() {
  if(!bleInitialized) nuosBleProvisioningBegin();
  if(connected) return;
  bleActive=true;
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Provisioning advertising STARTED by web UI");
}

void nuosBleProvisioningStop() {
  bleActive=false;
  if(bleInitialized) BLEDevice::getAdvertising()->stop();
  Serial.println("[BLE] Provisioning advertising STOPPED by web UI");
}

void nuosBleProvisioningLoop() {
  if(pendingWifi) {
    pendingWifi=false;
    AwsMqttConfig c;
    awsConfigLoad(c);
    c.wifiSsid=pendingSsid;
    c.wifiPassword=pendingPassword;
    if(!awsConfigSave(c)) {
      notify(responseChar,"{\"schema\":\"nuos.gateway.v2\",\"type\":\"wifi.status\",\"status\":\"failed\",\"message\":\"NVS_SAVE_FAILED\"}");
    } else {
      Serial.printf("[BLE] Applying Wi-Fi credentials for SSID=%s\n",pendingSsid.c_str());
      bool ok=awsMqttApplyWifiConfig(15000);
      notify(responseChar, ok ? "{\"schema\":\"nuos.gateway.v2\",\"type\":\"wifi.status\",\"status\":\"connected\"}" : "{\"schema\":\"nuos.gateway.v2\",\"type\":\"wifi.status\",\"status\":\"failed\"}");
      awsMqttFinishWifiApply(ok);
    }
    pendingSsid=""; pendingPassword="";
  }

  if(connected && millis()-lastStatusMs>5000) {
    lastStatusMs=millis();
    notify(statusChar,gatewayInfoJson());
  }
}

bool nuosBleProvisioningConnected() { return connected; }
bool nuosBleProvisioningActive() { return bleActive; }
