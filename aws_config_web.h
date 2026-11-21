#pragma once
#include <Arduino.h>
void awsWebBegin();
void awsWebLoop();
void zigbeeUiHandleH2Message(uint8_t type, const char *json);
