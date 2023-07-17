#include <CAN.h>

#define CS_PIN 10
#define IRQ_PIN 9
#define MCP2515_QUARTZ_MHZ 8  // Some MCP2515 boards have 8 MHz quartz.
#define SPI_MHZ 8
#define CAN_BAUD_RATE 500E3 // MX5 ND uses 500k baud rate hor HS CAN

// #define IS_DEBUG

#ifdef IS_DEBUG
#define CAN_SEND_INTERVAL 100
#else
#define CAN_SEND_INTERVAL 2
#endif

void setup() {
    Serial.begin(115200);

    u32 startTimeMs = millis();
    while (!Serial || millis() - startTimeMs < 1000);
    Serial.println("Started!");

    CAN.setClockFrequency(MCP2515_QUARTZ_MHZ * 1E6);
    CAN.setSPIFrequency(SPI_MHZ * 1E6);
    CAN.setPins(CS_PIN, IRQ_PIN);

    while (!CAN.begin(CAN_BAUD_RATE)) {
        Serial.println("Failed to connect to the CAN controller!");
        delay(1000);
    }

    Serial.println("CAN controller connected");
}

const u16 frameIdsToSend[] = {
    0x202, 0x215, 0x165, 0x240, 0x78, 0x86,
    0x415, 0x420,
    0x202, 0x215, 0x165, 0x240, 0x78, 0x86
};

const u16 framesCount = sizeof(frameIdsToSend) / sizeof(frameIdsToSend[0]);

void loop() {
    static u16 frameToSendIndex = 0;
    static u32 lastFrameSentTime = millis();

    if (!Serial) return;

    u32 currentTime = millis();
    if (currentTime - lastFrameSentTime < CAN_SEND_INTERVAL) return;
    lastFrameSentTime = currentTime;

    u16 frameId = frameIdsToSend[frameToSendIndex];
    u8 payload[8] = { 0 };
    generateFramePayload(frameId, payload);

    u8 sendResult = sendFrame(frameId, payload, 8);
    if (!sendResult) {
        Serial.print("Failed to send frame:0x");
        Serial.println(frameId, 16);
        return;
    }

#ifdef IS_DEBUG
    if (sendResult) {
        Serial.print("Sent frame: 0x");
        Serial.println(frameId, 16);
    }
#endif

    frameToSendIndex = (frameToSendIndex + 1) % framesCount;
}

boolean sendFrame(u16 id, u8* payload, u8 length) {
    if (!CAN.beginPacket(id)) {
        Serial.println("CAN.beginPacket() failed.");
        return false;
    }

    if (CAN.write(payload, length) != length) {
        Serial.println("CAN.write() failed.");
    }

    if (!CAN.endPacket()) {
        Serial.println("CAN.endPacket() failed.");
        return false;
    }

    return true;
}

void generateFramePayload(u16 pid, u8* payload) {
    switch (pid) {
    case 0x202: {
        u16 rpm = (880 + random(0, 100)) + ((millis() % 6000) < 3000 ? 2200 : 0);
        u16 raw = rpm * 4;
        payload[0] = (raw >> 8) & 0xFF;
        payload[1] = raw & 0xFF;

        break;
    }

    case 0x420: {
        u8 coolantTempC = 40 + millis() / 10000 + random(0, 2);
        u8 rawCoolantTemp = coolantTempC + 30;
        payload[0] = rawCoolantTemp;

        u8 airTempC = 22 + random(0, 2);
        u8 rawAirTemp = airTempC + 30;
        payload[7] = rawAirTemp;

        break;
    }

    default: {
        break;
    }
    }
}