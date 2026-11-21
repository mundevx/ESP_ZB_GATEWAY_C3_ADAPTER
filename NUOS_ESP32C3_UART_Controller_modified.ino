#include "aws_mqtt_gateway.h"
#include "aws_config_web.h"
#include "ble_provisioning.h"

#include <Arduino.h>
#include <ArduinoJson.h>

// ============================================================
// C3 ON-BOARD RGB STATUS LED
// ESP32-C3 Arduino core normally exposes RGB_BUILTIN (commonly GPIO8).
// If your PCB uses another RGB LED GPIO, change C3_RGB_LED_PIN only.
// ============================================================
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif
#define C3_RGB_LED_PIN RGB_BUILTIN

static void c3_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    rgbLedWrite(C3_RGB_LED_PIN, r, g, b);
}

static void c3_led_off()
{
    c3_led_set(0, 0, 0);
}

static void c3_led_pulse(uint8_t r, uint8_t g, uint8_t b, uint16_t ms)
{
    c3_led_set(r, g, b);
    delay(ms);
    c3_led_off();
}

// ============================================================
// UART PINS
// ============================================================

#define UART_RX_PIN     4
#define UART_TX_PIN     5

HardwareSerial MySerial(1);
#define UART_RX_BUFFER_SIZE 16384


// ============================================================
// PROTOCOL - MATCHES ESP32-H2 NUOS UART V2
// ============================================================
#define SOF             0x7E
#define ESCAPE          0x7D
#define XON             0x11
#define XOFF            0x13
#define PROTOCOL_VER    0x01

#define TYPE_COMMAND    0x01
#define TYPE_ACK        0x02
#define TYPE_RESPONSE   0x03
#define TYPE_EVENT      0x04
#define TYPE_ERROR      0x05

#define MAX_PAYLOAD     512

#define TYPE_FRAGMENT_START  0x06
#define TYPE_FRAGMENT_CONT   0x07
#define TYPE_FRAGMENT_END    0x08

#define FRAGMENT_HEADER_SIZE 9
#define FRAGMENT_DATA_SIZE   (MAX_PAYLOAD - FRAGMENT_HEADER_SIZE)
#define MAX_MESSAGE_SIZE     (32UL * 1024UL)
#define MAX_TX_MESSAGE_SIZE  (32UL * 1024UL)

// Static reassembly storage: avoids large-message malloc failures.
static uint8_t fragmentRxStorage[MAX_MESSAGE_SIZE + 1U];

// ============================================================
// RX STATE MACHINE - H2 V2 PHYSICAL FORMAT
// 7E | LEN_H | LEN_L | TYPE | SEQ | PAYLOAD | CHECKSUM
// LEN = TYPE + SEQ + PAYLOAD
// ============================================================
enum RX_STATE
{
    WAIT_SOF,
    READ_LEN_HIGH,
    READ_LEN_LOW,
    READ_TYPE,
    READ_SEQ,
    READ_PAYLOAD,
    READ_CHECKSUM
};

RX_STATE rxState = WAIT_SOF;

uint8_t  rxVersion = PROTOCOL_VER;
uint8_t  rxType = 0;
uint16_t rxSeq = 0;
uint16_t rxLength = 0;
uint16_t rxIndex = 0;
uint8_t  rxFrameSum = 0;
bool     rxFrameEscaped = false;
uint8_t  rxPayload[MAX_PAYLOAD + 1U];

// ============================================================
// SEQUENCE
// ============================================================

uint16_t sequenceNumber = 1;


// ============================================================
// ADAPTIVE UART PING/PONG
// ============================================================

#define UART_IDLE_PING_INTERVAL_MS 5000UL
#define UART_PING_TIMEOUT_MS       3000UL

static uint32_t lastMainUartActivityMs = 0;
static uint32_t pingSentMs = 0;
static bool pingAwaitingPong = false;
static uint16_t pingSequence = 0;

static void noteMainUartActivity()
{
    lastMainUartActivityMs = millis();
    pingAwaitingPong = false;
}

// Public C3-side H2 link status used by the web/API layer.
// H2 is considered online when a valid UART frame was received recently.
// A pending ping alone does not make H2 online; a valid RX frame does.
bool isH2Online()
{
    const uint32_t now = millis();
    constexpr uint32_t H2_ONLINE_TIMEOUT_MS = 10000UL;

    if (lastMainUartActivityMs == 0)
        return false;

    return (uint32_t)(now - lastMainUartActivityMs) <= H2_ONLINE_TIMEOUT_MS;
}


// ============================================================
// COMMAND ID
// ============================================================

uint32_t commandCounter = 1;


// ============================================================
// NEXT SEQUENCE NUMBER
// ============================================================

uint16_t getNextSequence()
{
    uint16_t seq = sequenceNumber++;

    if (sequenceNumber == 0)
    {
        sequenceNumber = 1;
    }

    return seq;
}


// ============================================================
// NEXT COMMAND ID
// ============================================================

void getNextCommandId(
    char *commandId,
    size_t size)
{
    snprintf(
        commandId,
        size,
        "cmd_%lu",
        (unsigned long)commandCounter++
    );
}


// ============================================================
// SEND FRAME - EXACTLY MATCHES H2
// ============================================================
static bool needsEscape(uint8_t b)
{
    return b == SOF || b == ESCAPE || b == XON || b == XOFF;
}

static void writeEscaped(uint8_t b)
{
    if (needsEscape(b)) {
        MySerial.write(ESCAPE);
        MySerial.write((uint8_t)(b ^ 0x20U));
    } else {
        MySerial.write(b);
    }
}

bool sendRawFrame(uint8_t type, uint16_t seq,
                  const uint8_t *payload, uint16_t payloadLength)
{
    if (payloadLength > MAX_PAYLOAD || (payloadLength && payload == nullptr))
        return false;

    // H2 uses an 8-bit sequence on the physical wire.
    uint8_t wireSeq = (uint8_t)seq;
    uint16_t frameLength = (uint16_t)(2U + payloadLength);
    uint8_t sum = (uint8_t)(type + wireSeq);
    for (uint16_t i = 0; i < payloadLength; ++i)
        sum = (uint8_t)(sum + payload[i]);
    uint8_t checksum = (uint8_t)(0xFFU - sum);

    Serial.printf("C3 UART TX FRAME: type=0x%02X seq=%u payload=%u bytes\n",
                  type, (unsigned)seq, (unsigned)payloadLength);
    c3_led_set(0, 0, 255);   // BLUE = physical UART TX

    MySerial.write(SOF);
    writeEscaped((uint8_t)(frameLength >> 8));
    writeEscaped((uint8_t)frameLength);
    writeEscaped(type);
    writeEscaped(wireSeq);
    for (uint16_t i = 0; i < payloadLength; ++i)
        writeEscaped(payload[i]);
    writeEscaped(checksum);
    MySerial.flush();
    c3_led_pulse(0, 0, 255, 20);
    return true;
}

bool sendFragmented(uint8_t originalType, uint16_t seq,
                    const uint8_t *data, size_t totalLength)
{
    if (!data || totalLength > MAX_TX_MESSAGE_SIZE)
        return false;

    const size_t chunkMax = MAX_PAYLOAD - FRAGMENT_HEADER_SIZE;
    uint16_t fragmentCount = (uint16_t)((totalLength + chunkMax - 1U) / chunkMax);
    if (fragmentCount == 0 || fragmentCount > 0xFFFFU)
        return false;

    static uint16_t messageId = 1;
    uint16_t id = messageId++;
    if (messageId == 0) messageId = 1;

    uint8_t fragmentPayload[MAX_PAYLOAD];
    for (uint16_t index = 0; index < fragmentCount; ++index) {
        size_t offset = (size_t)index * chunkMax;
        size_t remaining = totalLength - offset;
        uint16_t chunk = (uint16_t)(remaining > chunkMax ? chunkMax : remaining);

        uint8_t flags = 0;
        if (index == 0) flags |= 0x01;
        if (index == fragmentCount - 1U) flags |= 0x02;

        fragmentPayload[0] = originalType;
        fragmentPayload[1] = flags;
        fragmentPayload[2] = (uint8_t)(id >> 8);
        fragmentPayload[3] = (uint8_t)id;
        fragmentPayload[4] = (uint8_t)(index >> 8);
        fragmentPayload[5] = (uint8_t)index;
        fragmentPayload[6] = (uint8_t)(fragmentCount >> 8);
        fragmentPayload[7] = (uint8_t)fragmentCount;
        fragmentPayload[8] = 0x01;
        memcpy(&fragmentPayload[FRAGMENT_HEADER_SIZE], data + offset, chunk);

        uint8_t frameType = index == 0 ? TYPE_FRAGMENT_START :
                            (index == fragmentCount - 1U ? TYPE_FRAGMENT_END : TYPE_FRAGMENT_CONT);
        if (!sendRawFrame(frameType, seq, fragmentPayload,
                          (uint16_t)(FRAGMENT_HEADER_SIZE + chunk)))
            return false;

        // Deliberate pacing for long transfers.
        delay(5);
    }

    Serial.printf("C3 TX fragmented: %u bytes, %u fragments, ID=%u\n",
                  (unsigned)totalLength, fragmentCount, id);
    return true;
}

bool sendFrame(uint8_t type, uint16_t seq, const char *json)
{
    if (json == nullptr)
        return false;
    size_t payloadLength = strlen(json);
    if (payloadLength > MAX_TX_MESSAGE_SIZE)
        return false;

    if (payloadLength <= MAX_PAYLOAD)
        return sendRawFrame(type, seq, (const uint8_t *)json, (uint16_t)payloadLength);

    Serial.printf("UART payload large: %u bytes, fragmenting...\n", (unsigned)payloadLength);
    return sendFragmented(type, seq, (const uint8_t *)json, payloadLength);
}

// ============================================================
// SEND COMMAND
// ============================================================

uint16_t sendCommand(
    const char *json)
{
    uint16_t seq =
        getNextSequence();

    Serial.println();
    Serial.println(
        "--------------------------------"
    );

    Serial.println(
        "C3 SEND COMMAND"
    );

    Serial.print(
        "SEQ: "
    );

    Serial.println(seq);

    Serial.print(
        "Payload: "
    );

    Serial.println(json);


    sendFrame(
        TYPE_COMMAND,
        seq,
        json
    );


    Serial.println(
        "--------------------------------"
    );


    return seq;
}


// ============================================================
// SEND DEVICE LIST COMMAND
// ============================================================

void requestAllDevices()
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.list\","
        "\"arguments\":{}"
        "}",

        commandId,
        commandId
    );


    sendCommand(json);
}


// ============================================================
// SEND UNCONFIGURED DEVICE QUERY
// ============================================================

void requestUnconfiguredDevices()
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.list\","
        "\"arguments\":{"
            "\"filter\":{"
                "\"configurationState\":\"unconfigured\""
            "}"
        "}"
        "}",

        commandId,
        commandId
    );


    sendCommand(json);
}


// ============================================================
// QUERY ALL DEVICE STATUS
// ============================================================

void requestAllDeviceStatus()
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.status.list\","
        "\"arguments\":{}"
        "}",

        commandId,
        commandId
    );


    sendCommand(json);
}


// ============================================================
// QUERY SINGLE DEVICE
// ============================================================

void requestDeviceInfo(
    const char *deviceId)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.get\","
        "\"deviceId\":\"%s\","
        "\"arguments\":{}"
        "}",

        commandId,
        commandId,
        deviceId
    );


    sendCommand(json);
}


// ============================================================
// QUERY SINGLE DEVICE STATUS
// ============================================================

void requestDeviceStatus(
    const char *deviceId)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.status.get\","
        "\"deviceId\":\"%s\","
        "\"arguments\":{}"
        "}",

        commandId,
        commandId,
        deviceId
    );


    sendCommand(json);
}


// ============================================================
// CONFIGURE DEVICE
// ============================================================

void configureDevice(
    const char *deviceId,
    uint8_t roomId,
    uint8_t moduleId,
    const char *roomName,
    const char *deviceName)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[768];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.configure\","
        "\"deviceId\":\"%s\","
        "\"arguments\":{"
            "\"roomId\":%u,"
            "\"moduleId\":%u,"
            "\"roomName\":\"%s\","
            "\"deviceName\":\"%s\""
        "}"
        "}",

        commandId,
        commandId,

        deviceId,

        roomId,
        moduleId,

        roomName,
        deviceName
    );


    sendCommand(json);
}


// ============================================================
// CONFIGURE ENDPOINT
// ============================================================

void configureEndpoint(
    const char *deviceId,
    uint8_t endpointId,
    const char *endpointName)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"endpoint.configure\","
        "\"deviceId\":\"%s\","
        "\"endpointId\":\"%u\","
        "\"arguments\":{"
            "\"name\":\"%s\""
        "}"
        "}",

        commandId,
        commandId,

        deviceId,
        endpointId,

        endpointName
    );


    sendCommand(json);
}


// ============================================================
// REMOVE DEVICE
// ============================================================

void removeDevice(
    const char *deviceId)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"device.remove\","
        "\"deviceId\":\"%s\","
        "\"arguments\":{}"
        "}",

        commandId,
        commandId,

        deviceId
    );


    sendCommand(json);
}


// ============================================================
// START COMMISSIONING
// ============================================================

void startCommissioning(
    uint16_t duration)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"commissioning.start\","
        "\"arguments\":{"
            "\"duration\":%u"
        "}"
        "}",

        commandId,
        commandId,
        duration
    );


    sendCommand(json);
}


// ============================================================
// STOP COMMISSIONING
// ============================================================

void stopCommissioning()
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[512];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"command\":\"commissioning.stop\","
        "\"arguments\":{}"
        "}",

        commandId,
        commandId
    );


    sendCommand(json);
}


// ============================================================
// CONTROL ON/OFF
// ============================================================

void setOnOff(
    const char *deviceId,
    uint8_t endpointId,
    bool on)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[600];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"deviceId\":\"%s\","
        "\"endpointId\":\"%u\","
        "\"capability\":\"on_off\","
        "\"command\":\"set\","
        "\"arguments\":{"
            "\"on\":%s"
        "}"
        "}",

        commandId,
        commandId,

        deviceId,
        endpointId,

        on ? "true" : "false"
    );


    sendCommand(json);
}


// ============================================================
// CONTROL BRIGHTNESS
// ============================================================

void setBrightness(
    const char *deviceId,
    uint8_t endpointId,
    uint8_t brightness)
{
    char commandId[32];

    getNextCommandId(
        commandId,
        sizeof(commandId)
    );


    char json[600];

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"type\":\"command\","
        "\"commandId\":\"%s\","
        "\"idempotencyKey\":\"%s\","
        "\"deviceId\":\"%s\","
        "\"endpointId\":\"%u\","
        "\"capability\":\"brightness\","
        "\"command\":\"set\","
        "\"arguments\":{"
            "\"brightness\":%u"
        "}"
        "}",

        commandId,
        commandId,

        deviceId,
        endpointId,

        brightness
    );


    sendCommand(json);
}


// ============================================================


// ============================================================
// UART MESSAGE PROCESSING HANDLERS
// ============================================================

void processEvent(
    const char *json)
{
    Serial.println();
    Serial.println(
        "******** ZIGBEE EVENT ********"
    );

    Serial.println(json);

    // --------------------------------------------------------
    // PHYSICAL GATEWAY PROVISIONING REQUEST FROM H2
    // --------------------------------------------------------
    // H2 sends this after a 5-second long press:
    //   mode=wifi -> stop BLE and start Wi-Fi provisioning AP
    //   mode=ble  -> stop Wi-Fi and start BLE provisioning
    // --------------------------------------------------------
    if (strstr(json, "\"eventType\":\"gateway.provisioning\"") != nullptr)
    {
        StaticJsonDocument<512> provisioning;
        DeserializationError err = deserializeJson(provisioning, json);
        if (!err)
        {
            const char *mode = provisioning["mode"] | "";
            uint16_t pressSeconds = provisioning["pressDurationSec"] | 0;

            Serial.printf("[H2 BUTTON] Provisioning request: mode=%s, hold=%us\n",
                          mode, (unsigned)pressSeconds);

            if (!strcmp(mode, "wifi"))
            {
                Serial.println("[H2 BUTTON] -> START WIFI PROVISIONING");
                nuosBleProvisioningStop();
                awsMqttStartWifiProvisioning();
            }
            else if (!strcmp(mode, "ble"))
            {
                Serial.println("[H2 BUTTON] -> START BLE PROVISIONING");
                awsMqttStopWifiForBle();
                nuosBleProvisioningStart();
            }
            else
            {
                Serial.printf("[H2 BUTTON] Unknown provisioning mode: %s\n", mode);
            }
        }

        Serial.println("*******************************");
        return;
    }


    // --------------------------------------------------------
    // DEVICE ADDED
    // --------------------------------------------------------

    if (strstr(
            json,
            "\"eventType\":\"device.added\"")
        != nullptr)
    {
        Serial.println(
            "EVENT = DEVICE ADDED"
        );

        /*
         * Here:
         *
         * 1. Parse device information
         * 2. Add device to cloud database
         * 3. Check configuration.state
         *
         * If:
         *
         * configurationState = unconfigured
         *
         * show device in "New Devices".
         */
    }


    // --------------------------------------------------------
    // DEVICE REMOVED
    // --------------------------------------------------------

    else if (strstr(
                json,
                "\"eventType\":\"device.removed\"")
             != nullptr)
    {
        Serial.println(
            "EVENT = DEVICE REMOVED"
        );

        /*
         * Remove/mark removed in cloud DB.
         */
    }


    // --------------------------------------------------------
    // DEVICE REJOINED
    // --------------------------------------------------------

    else if (strstr(
                json,
                "\"eventType\":\"device.rejoined\"")
             != nullptr)
    {
        Serial.println(
            "EVENT = DEVICE REJOINED"
        );

        /*
         * Update short address.
         *
         * Do NOT create a new device.
         */
    }


    // --------------------------------------------------------
    // PROPERTY CHANGED
    // --------------------------------------------------------

    else if (strstr(
                json,
                "\"eventType\":\"property.changed\"")
             != nullptr)
    {
        Serial.println(
            "EVENT = PROPERTY CHANGED"
        );

        /*
         * Update cloud reported state.
         */
    }


    // --------------------------------------------------------
    // CONNECTIVITY
    // --------------------------------------------------------

    else if (strstr(
                json,
                "\"eventType\":\"connectivity.changed\"")
             != nullptr)
    {
        Serial.println(
            "EVENT = CONNECTIVITY CHANGED"
        );

        /*
         * Update online/offline status.
         */
    }


    // --------------------------------------------------------
    // CONFIGURATION CHANGED
    // --------------------------------------------------------

    else if (strstr(
                json,
                "\"eventType\":\"device.configuration_changed\"")
             != nullptr)
    {
        Serial.println(
            "EVENT = CONFIGURATION CHANGED"
        );
    }


    Serial.println(
        "*******************************"
    );
}

void processACK(
    const char *json)
{
    Serial.println();
    Serial.println(
        "******** UART ACK ********"
    );

    Serial.println(json);

    /*
     * Example:
     *
     * {
     *   "type":"ack",
     *   "commandId":"cmd_12",
     *   "status":"accepted"
     * }
     *
     * ACK means H2 received the command.
     *
     * It does NOT necessarily mean that the
     * Zigbee operation has completed.
     */

    if (strstr(
            json,
            "\"status\":\"accepted\"")
        != nullptr)
    {
        Serial.println(
            "Command accepted by H2"
        );
    }

    else if (strstr(
                json,
                "\"status\":\"rejected\"")
             != nullptr)
    {
        Serial.println(
            "Command rejected by H2"
        );
    }

    Serial.println(
        "***************************"
    );
}

void processResponse(
    const char *json)
{
    if (json != nullptr && strstr(json, "\"status\":\"pong\"") != nullptr)
    {
        pingAwaitingPong = false;
        lastMainUartActivityMs = millis();
        Serial.println("C3 UART: pong received - heartbeat complete");
    }

    Serial.println();
    Serial.println(
        "******** UART RESPONSE ********"
    );

    Serial.println(json);

    /*
     * This can contain:
     *
     * device.list
     * device.get
     * device.status.list
     * device.status.get
     *
     * The complete JSON should normally be
     * forwarded to your cloud/MQTT layer.
     */

    if (strstr(
            json,
            "\"status\":\"completed\"")
        != nullptr)
    {
        Serial.println(
            "Response completed"
        );
    }

    // Forward the complete H2 response to the Web/UI cache handler.
    // This also handles large fragmented device.list responses after reassembly.
    zigbeeUiHandleH2Message(TYPE_RESPONSE, json);

    Serial.println(
        "********************************"
    );
}

void processError(
    const char *json)
{
    Serial.println();
    Serial.println(
        "******** UART ERROR ********"
    );

    Serial.println(json);

    Serial.println(
        "*****************************"
    );
}

void processFrame()
{
    rxPayload[rxLength] = '\0';


    // --------------------------------------------------------
    // DEBUG
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "C3 RECEIVED FRAME"
    );

    Serial.print(
        "Type: "
    );

    Serial.println(
        rxType
    );

    Serial.print(
        "SEQ: "
    );

    Serial.println(
        rxSeq
    );

    Serial.print(
        "Payload: "
    );

    Serial.println(
        (char *)rxPayload
    );


    // --------------------------------------------------------
    // DISPATCH
    // --------------------------------------------------------

    switch (rxType)
    {
        case TYPE_ACK:

            processACK(
                (char *)rxPayload
            );

            break;


        case TYPE_RESPONSE:

            processResponse(
                (char *)rxPayload
            );

            break;


        case TYPE_EVENT:

            processEvent(
                (char *)rxPayload
            );

            break;


        case TYPE_ERROR:

            processError(
                (char *)rxPayload
            );

            break;


        default:

            Serial.println(
                "Unknown frame type"
            );

            break;
    }


    Serial.println(
        "================================"
    );
}

// FRAGMENT REASSEMBLY
// ============================================================

struct FragmentRxContext
{
    bool active;
    uint16_t messageId;
    uint16_t fragmentCount;
    uint16_t nextFragment;
    uint16_t seq;
    uint8_t originalType;
    size_t totalLength;
    uint8_t *buffer;
};

FragmentRxContext fragmentRx =
{
    false, 0, 0, 0, 0, 0, 0, fragmentRxStorage
};

bool isFragmentType(uint8_t type)
{
    return
        type == TYPE_FRAGMENT_START ||
        type == TYPE_FRAGMENT_CONT ||
        type == TYPE_FRAGMENT_END;
}

void resetFragmentRx()
{
    fragmentRx.active = false;
    fragmentRx.messageId = 0;
    fragmentRx.fragmentCount = 0;
    fragmentRx.nextFragment = 0;
    fragmentRx.seq = 0;
    fragmentRx.originalType = 0;
    fragmentRx.totalLength = 0;
    fragmentRx.buffer = fragmentRxStorage;
}

void processCompleteMessage(
    uint8_t type,
    uint16_t seq,
    const uint8_t *data,
    size_t length)
{
    if (length > MAX_MESSAGE_SIZE)
        return;

    // Existing processFrame() uses rxPayload/rxLength.
    if (length <= MAX_PAYLOAD)
    {
        memcpy(rxPayload, data, length);
        rxLength = (uint16_t)length;
        rxType = type;
        rxSeq = seq;
        rxPayload[length] = '\0';

        processFrame();
        return;
    }

    // Large JSON is already in the static reassembly buffer.
    // Dispatch it directly; do not malloc/copy another large buffer.
    const char *json = (const char *)data;

    Serial.println();
    Serial.println("================================");
    Serial.println("C3 RECEIVED LARGE FRAME");
    Serial.print("Type: ");
    Serial.println(type);
    Serial.print("SEQ: ");
    Serial.println(seq);
    Serial.print("Length: ");
    Serial.println((unsigned)length);
    Serial.println("Payload:");
    Serial.println(json);
    Serial.println("================================");

    if (type == TYPE_EVENT)
        processEvent(json);
    else if (type == TYPE_RESPONSE)
        processResponse(json);
    else if (type == TYPE_ACK)
        processACK(json);
    else if (type == TYPE_ERROR)
        processError(json);

}

void processFragment(
    uint8_t frameType,
    uint16_t seq,
    const uint8_t *data,
    uint16_t length)
{
    if (length < FRAGMENT_HEADER_SIZE)
    {
        Serial.println("ERROR: fragment too short");
        c3_led_pulse(255, 0, 0, 120);
        resetFragmentRx();
        return;
    }

    // H2 fragment header: type, flags, messageId(2), index(2), count(2), version
    uint8_t originalType = data[0];
    uint8_t flags = data[1];
    uint16_t messageId = ((uint16_t)data[2] << 8) | data[3];
    uint16_t fragmentIndex = ((uint16_t)data[4] << 8) | data[5];
    uint16_t fragmentCount = ((uint16_t)data[6] << 8) | data[7];
    uint8_t headerVersion = data[8];

    uint16_t dataLength = length - FRAGMENT_HEADER_SIZE;

    if (headerVersion != 0x01 ||
        fragmentCount == 0 ||
        fragmentIndex >= fragmentCount ||
        originalType == 0 ||
        isFragmentType(originalType))
    {
        Serial.println("ERROR: invalid fragment header");
        c3_led_pulse(255, 0, 0, 120);
        resetFragmentRx();
        return;
    }

    if (frameType == TYPE_FRAGMENT_START)
    {
        resetFragmentRx();

        size_t allocationSize =
            (size_t)fragmentCount * FRAGMENT_DATA_SIZE;

        if (allocationSize > MAX_MESSAGE_SIZE)
        {
            Serial.println("ERROR: fragment allocation too large");
            c3_led_pulse(255, 0, 0, 120);
            return;
        }

        fragmentRx.buffer = fragmentRxStorage;
        fragmentRx.active = true;
        fragmentRx.messageId = messageId;
        fragmentRx.fragmentCount = fragmentCount;
        fragmentRx.nextFragment = 0;
        fragmentRx.seq = seq;
        fragmentRx.originalType = originalType;
        fragmentRx.totalLength = 0;

        Serial.printf("C3 FRAGMENT START: id=%u count=%u originalType=0x%02X flags=0x%02X\n",
                      messageId, fragmentCount, originalType, flags);
    }

    if (!fragmentRx.active ||
        fragmentRx.messageId != messageId ||
        fragmentRx.fragmentCount != fragmentCount ||
        fragmentRx.seq != seq ||
        fragmentRx.originalType != originalType ||
        fragmentRx.nextFragment != fragmentIndex)
    {
        Serial.println("ERROR: fragment sequence mismatch");
        c3_led_pulse(255, 0, 0, 120);
        resetFragmentRx();
        return;
    }

    if (fragmentRx.totalLength + dataLength > MAX_MESSAGE_SIZE)
    {
        Serial.println("ERROR: reassembled message too large");
        c3_led_pulse(255, 0, 0, 120);
        resetFragmentRx();
        return;
    }

    memcpy(&fragmentRx.buffer[fragmentRx.totalLength],
           &data[FRAGMENT_HEADER_SIZE],
           dataLength);

    fragmentRx.totalLength += dataLength;
    fragmentRx.nextFragment++;

    if (frameType == TYPE_FRAGMENT_END)
    {
        if (fragmentIndex != fragmentCount - 1 ||
            fragmentRx.nextFragment != fragmentCount)
        {
            Serial.println("ERROR: incomplete fragmented message");
            c3_led_pulse(255, 0, 0, 120);
            resetFragmentRx();
            return;
        }

        fragmentRx.buffer[fragmentRx.totalLength] = '\0';

        Serial.printf("C3 RX reassembled: %u bytes, %u fragments, ID=%u\n",
                      (unsigned)fragmentRx.totalLength, fragmentCount, messageId);
        c3_led_pulse(0, 255, 255, 80);   // CYAN = complete large message

        processCompleteMessage(fragmentRx.originalType,
                               fragmentRx.seq,
                               fragmentRx.buffer,
                               fragmentRx.totalLength);

        resetFragmentRx();
    }
}

// ============================================================
// RECEIVE FRAME - EXACTLY MATCHES H2
// ============================================================
void receiveFrame()
{
    while (MySerial.available()) {
        uint8_t raw = (uint8_t)MySerial.read();

        if (rxState != WAIT_SOF && rxFrameEscaped) {
            raw ^= 0x20U;
            rxFrameEscaped = false;
        } else if (rxState != WAIT_SOF && raw == ESCAPE) {
            rxFrameEscaped = true;
            continue;
        }

        if (rxState != WAIT_SOF && raw == SOF) {
            rxState = READ_LEN_HIGH;
            rxIndex = 0;
            rxFrameSum = 0;
            rxFrameEscaped = false;
            continue;
        }

        switch (rxState) {
        case WAIT_SOF:
            if (raw == SOF) {
                rxState = READ_LEN_HIGH;
                rxIndex = 0;
                rxFrameSum = 0;
                rxFrameEscaped = false;
            }
            break;

        case READ_LEN_HIGH:
            rxLength = (uint16_t)raw << 8;
            rxState = READ_LEN_LOW;
            break;

        case READ_LEN_LOW:
            rxLength |= raw;
            if (rxLength < 2U || rxLength > (MAX_PAYLOAD + 2U)) {
                Serial.printf("ERROR: invalid UART frame length=%u\n", rxLength);
                rxState = WAIT_SOF;
            } else {
                rxIndex = 0;
                rxState = READ_TYPE;
            }
            break;

        case READ_TYPE:
            rxType = raw;
            rxFrameSum = raw;
            rxState = READ_SEQ;
            break;

        case READ_SEQ:
            rxSeq = raw;
            rxFrameSum = (uint8_t)(rxFrameSum + raw);
            rxState = (rxLength == 2U) ? READ_CHECKSUM : READ_PAYLOAD;
            break;

        case READ_PAYLOAD:
            if (rxIndex >= MAX_PAYLOAD) {
                rxState = WAIT_SOF;
                break;
            }
            rxPayload[rxIndex++] = raw;
            rxFrameSum = (uint8_t)(rxFrameSum + raw);
            if (rxIndex >= rxLength - 2U)
                rxState = READ_CHECKSUM;
            break;

        case READ_CHECKSUM: {
            uint8_t expected = (uint8_t)(0xFFU - rxFrameSum);
            if (raw != expected) {
                Serial.printf("ERROR: UART checksum mismatch type=%u seq=%u rx=0x%02X calc=0x%02X\n",
                              rxType, rxSeq, raw, expected);
                c3_led_pulse(255, 0, 0, 120);   // RED = corrupted RX frame
                rxState = WAIT_SOF;
                break;
            }

            rxVersion = PROTOCOL_VER;
            noteMainUartActivity();
            c3_led_pulse(0, 255, 0, 35);   // GREEN = valid H2 RX frame
            Serial.printf("C3 UART RX VALID FRAME: type=0x%02X seq=%u payload=%u bytes\n",
                          rxType, (unsigned)rxSeq, (unsigned)(rxLength - 2U));
            uint16_t payloadLength = (uint16_t)(rxLength - 2U);

            if (isFragmentType(rxType)) {
                processFragment(rxType, rxSeq, rxPayload, payloadLength);
            } else {
                processCompleteMessage(rxType, rxSeq, rxPayload, payloadLength);
            }
            rxState = WAIT_SOF;
            break;
        }

        default:
            rxState = WAIT_SOF;
            break;
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    c3_led_set(255, 255, 255);
    delay(150);
    c3_led_set(0, 0, 255);

    MySerial.setRxBufferSize(UART_RX_BUFFER_SIZE);

    MySerial.begin(
        115200,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    // Start C3 Wi-Fi provisioning/STA service and register the HTTP server.
    // The HTTP server is compiled into flash; no LittleFS mount is required.
    awsMqttBegin();
    awsWebBegin();

    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "NUOS ESP32-C3 UART Gateway"
    );

    Serial.println(
        "Protocol Version: 1"
    );

    Serial.println(
        "Physical Payload: 512 bytes"
    );

    Serial.println(
        "Large Message: Fragmentation enabled"
    );

    Serial.println(
        "Baud Rate: 115200"
    );

    Serial.println(
        "================================"
    );
    Serial.printf("C3 UART PINS: TX=GPIO%d RX=GPIO%d\n", UART_TX_PIN, UART_RX_PIN);
    Serial.printf("C3 RGB LED: GPIO%d\n", C3_RGB_LED_PIN);
    Serial.println("C3 UART DIAGNOSTIC: immediate link.ping will be transmitted");

    // Force a physical UART transmission immediately after boot.
    delay(500);
    const char *startupPing =
        "{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"c3_startup_link\",\"command\":\"link.ping\",\"arguments\":{}}";
    Serial.println("C3 UART: STARTUP link.ping TX");
    uint16_t startupSeq = getNextSequence();
    if (sendFrame(TYPE_COMMAND, startupSeq, startupPing))
    {
        pingSentMs = millis();
        pingAwaitingPong = true;
        c3_led_pulse(160, 0, 255, 120);   // PURPLE = ping sent
    }
    else
    {
        c3_led_pulse(255, 0, 0, 250);
    }
}


// ============================================================
// ADAPTIVE LINK PING SERVICE
// ============================================================
static void serviceAdaptivePing()
{
    uint32_t now = millis();

    if (pingAwaitingPong)
    {
        if ((uint32_t)(now - pingSentMs) >= UART_PING_TIMEOUT_MS)
        {
            pingAwaitingPong = false;
            c3_led_set(255, 0, 0);
            Serial.println("C3 UART: PING TIMEOUT - H2 did not answer");
        }
        return;
    }

    if (fragmentRx.active)
        return;

    if ((uint32_t)(now - lastMainUartActivityMs) < UART_IDLE_PING_INTERVAL_MS)
        return;

    uint16_t seq = getNextSequence();
    const char *pingJson =
        "{\"schema\":\"nuos.gateway.v2\",\"type\":\"command\",\"requestId\":\"c3_uart_link\",\"command\":\"link.ping\",\"arguments\":{}}";

    Serial.printf("C3 UART: IDLE -> sending link.ping seq=%u\n", seq);
    if (sendFrame(TYPE_COMMAND, seq, pingJson))
    {
        pingSentMs = now;
        pingAwaitingPong = true;
        c3_led_pulse(160, 0, 255, 100);
    }
    else
    {
        c3_led_pulse(255, 0, 0, 200);
    }
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Keep Wi-Fi state machine and HTTP server alive.
    // --------------------------------------------------------
    awsMqttLoop();
    awsWebLoop();
    nuosBleProvisioningLoop();

    // --------------------------------------------------------
    // Always process incoming H2 messages
    // --------------------------------------------------------
    receiveFrame();
    serviceAdaptivePing();
    // --------------------------------------------------------
    // TEST COMMAND
    //
    // Currently every 30 seconds request
    // unconfigured devices.
    // --------------------------------------------------------
    static uint32_t previousMillis = 0;

    if (millis() -
        previousMillis >= 30000)
    {
        previousMillis =
            millis();


        requestUnconfiguredDevices();
    }
}