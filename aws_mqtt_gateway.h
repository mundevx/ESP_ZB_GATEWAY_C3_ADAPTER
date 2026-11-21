#pragma once
#include <Arduino.h>

void awsMqttBegin();
void awsMqttLoop();
void awsMqttPublishH2Message(uint8_t h2Type, const char *h2Json);
bool awsMqttIsConnected();

// Apply the saved Wi-Fi configuration immediately.
// Returns true when STA obtained an IP within timeoutMs.
bool awsMqttApplyWifiConfig(uint32_t timeoutMs = 15000);

// Finish the STA/AP transition after the HTTP response has been sent.
void awsMqttFinishWifiApply(bool staConnected);

// Physical H2 button control: start/stop Wi-Fi provisioning mode.
void awsMqttStartWifiProvisioning();
void awsMqttStopWifiForBle();
