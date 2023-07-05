#include <CAN.h>

#define CS_PIN 10
#define IRQ_PIN 9
#define MCP2515_QUARTZ_MHZ 16  // Some MCP2515 boards have 8 MHz quartz.
#define SPI_MHZ 8
#define CAN_BAUD_RATE 500E3 // MX5 ND uses 500k baud rate hor HS CAN

#define IS_DEBUG

#ifdef IS_DEBUG
#define CAN_SEND_INTERVAL 1000
#else
#define CAN_SEND_INTERVAL 10
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



// 0x40 and 0x41 are intentionally duplicated in this array as they are sent 100 times
// per second (double that for other PIDs) in the real car.
const u16 frameIdsToSend[] = {
    // These are sent 100 times per second:
    0x40, 0x41,
    // These are sent 50 times per second, part 1:
    0x118, 0x138, 0x139, 0x13B, 0x13C,
    // These are sent 100 times per second (duplicates):
    0x40, 0x41,
    // These are sent 50 times per second, part 2:
    0x143, 0x146,
    // TODO: These are actually sent less frequently than 50 times per second:
    0x241, // 20 times per second
    0x345, // 10 times per second
};

const u16 framesCount = sizeof(frameIdsToSend) / sizeof(frameIdsToSend[0]);

void loop() {
    static u16 frameToSendIndex = 0;
    static u32 lastFrameSentTime = millis();

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

    frameToSendIndex++;
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
    case 0x40: {
        u8 acceleratorPedalPercent = 42;
        payload[4] = acceleratorPedalPercent * 255 / 100;
        payload[5] = payload[4];
        payload[6] = payload[4];

        // The clutch pedal has two sensors:
        // - 0% and >0% (used here)
        // - 100% and <100% (haven't found yet)
        // TODO: Find where data from the second sensor is.
        bool clutchDown = false;
        payload[1] = (clutchDown ? 0x80 : 0x00);

        // RPMs are believed to be encoded with just 14 bits.
        u16 rpm = 3456;
        payload[2] = rpm & 0xFF;
        payload[3] = (rpm >> 8) & 0x3F;
        break;
    }

    case 0x138: {
        // Pretend that the steering wheel is turned by 123 degrees to the left
        int16_t steeringAngleDegrees = -123;
        int16_t steeringValue = steeringAngleDegrees * 10;
        payload[2] = steeringValue & 0xFF;
        payload[3] = (steeringValue >> 8) & 0xFF;

        // TODO: Verify the scale for this value. The current scale is suspicious,
        // but matches real-world testing so far. Need to go to a skid pad to
        // verify for sure.
        float yawRateDegreesPerSecond = -12.3;
        int16_t yawRateValue = (int16_t)(-yawRateDegreesPerSecond / 0.2725);
        payload[4] = yawRateValue & 0xFF;
        payload[5] = (yawRateValue >> 8) & 0xFF;
        break;
    }

    case 0x139: {
        u16 speedMetersPerSecond = 10;  // 36 km/h, ~22.4 mph.
        // The encoding seems to be roughly radians per second x100.
        // The coefficient was tuned by comparing the values against an external
        // GPS from a session where I drove in a straight line on a highway at
        // constant speed on cruise control.
        u16 speedValue = (u16)(speedMetersPerSecond * 63.72);
        payload[2] = speedValue & 0xFF;
        payload[3] = (speedValue >> 8) & 0xFF;

        // The units used for the master brake cylinder pressure are believed to
        // be 1/128 kPa.
        float brakePressureKpa = 1024;
        payload[4] = 0x0C;
        payload[5] = (u8)(brakePressureKpa / 128);
        break;
    }

    case 0x345: {
        u8 oilTempC = 100;
        payload[3] = oilTempC + 40;

        u8 coolantTempC = 90;
        payload[4] = coolantTempC + 40;
        break;
    }

    default: {
        break;
    }
    }
}