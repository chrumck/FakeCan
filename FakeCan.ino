#include <CAN.h>

// This is a demo program that sends messages over the CAN bus in
// a way that resembles real messages you can receive if you listen
// to messages on the CAN bus of a 2022 Toyota GR86.
//
// DO NOT USE IT IN THE CAN NETWORK OF A REAL VEHICLE as it can cause unexpected
// side effects.
//
// It was tested on Arduino Uno, Arduino Micro, Adafruit Feather nRF52832 and
// Adafruit ItsyBitsy nRF52840 Express, and should be trivial to tweak to
// support pretty much any other board with SPI.
//
// Connections:
//  MCP | BOARD
//  INT | Not used, can connect to Pin 9
//  SCK | SCK
//   SI | MO
//   SO | MI
//   CS | Pin 7
//  GND | GND
//  VCC | 3.3V

#define CS_PIN 7
#define IRQ_PIN 9
#define QUARTZ_MHZ 16  // Some MCP2515 boards have 8 MHz quartz.
#define SPI_MHZ 8
#define CAN_BAUD_RATE 500e3 // MX5 ND uses 500k baud rate hor HS CAN

void setup() {
    Serial.begin(115200);

    u32 startTimeMs = millis();
    while (!Serial && millis() - startTimeMs < 1000);
    Serial.println("Started!");


    CAN.setClockFrequency(QUARTZ_MHZ * 1E6);
    CAN.setSPIFrequency(SPI_MHZ * 1E6);
    CAN.setPins(CS_PIN, IRQ_PIN);

    // Subaru BRZ uses a 500k baud rate.
    while (!CAN.begin(CAN_BAUD_RATE)) {
        Serial.println("Failed to connect to the CAN controller!");
        delay(1000);
    }

    Serial.println("CAN controller connected");
}

class FakeTpmsEcu {
public:
    FakeTpmsEcu() {
        hasContinuationFrame = false;
        originalFrameAcked = false;
    }

    void scheduleNextFrame(u16 pid, u8* data, u8 length) {
        if (hasContinuationFrame) {
            Serial.print("Scheduling new frame, even though there is one pending already. Original was ");
            if (originalFrameAcked) {
                Serial.println("acked.");
            }
            else {
                Serial.println("NOT acked.");
            }
        }
        hasContinuationFrame = true;
        originalFrameAcked = false;
        nextFramePid = pid;
        memcpy(nextFrameData, data, length);
        nextFrameLength = length;
    }

    void handleFrameAck(u8 separationTimeMillis = 0) {
        if (!hasContinuationFrame) return;

        originalFrameAcked = true;
        nextFrameTimestampMillis = millis() + separationTimeMillis;
    }

    void sendNextFrameIfNeeded() {
        if (!hasContinuationFrame || !originalFrameAcked) return;

        // Unsigned math magic to check if "timeDiff" is "negative":
        u32 timeDiff = millis() - nextFrameTimestampMillis;
        if (timeDiff >> (8 * sizeof(timeDiff) - 1)) {
            return;
        }

        sendFrame(nextFramePid, nextFrameData, nextFrameLength);
        hasContinuationFrame = false;
        originalFrameAcked = false;
    }

private:
    bool hasContinuationFrame;
    u16 nextFramePid;
    u8 nextFrameData[8];
    u8 nextFrameLength;

    bool originalFrameAcked;
    u32 nextFrameTimestampMillis;
} fakeTpmsEcu;

// Forward declaration for a helper.
void receiveFrames();
void generatePayload(u16 pid, u8* payload);
boolean sendFrame(u16 pid, u8* payload, u8 len);

// Many PIDs are sent 50 times per second.
const u16 cyclesPerSecond = 50;

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

const u16 messagesPerCycle = sizeof(frameIdsToSend) / sizeof(frameIdsToSend[0]);
const u16 messagesPerSecond = cyclesPerSecond * messagesPerCycle;

void loop() {
    u32 firstMessageSentTime = micros();
    u32 messagesSentCount = 0;

    for (int i = 0; i < messagesPerCycle; i++, messagesSentCount++) {
        receiveFrames();
        fakeTpmsEcu.sendNextFrameIfNeeded();

        u32 nextMessageTime = firstMessageSentTime + (messagesSentCount * 1e6) / messagesPerSecond;
        if (micros() - nextMessageTime >= 0) continue;

        u16 frameId = frameIdsToSend[i];
        u8 payload[8] = { 0 };
        generatePayload(frameId, payload);
        if (!sendFrame(frameId, payload, 8)) Serial.println("Failed to send a message");

        delayMicroseconds(10);
    }
}

boolean sendFrame(u16 id, u8* payload, u8 len) {
    if (!CAN.beginPacket(id)) {
        Serial.println("beginPacket() failed.");
        return false;
    }

    CAN.write(payload, len);

    if (!CAN.endPacket()) {
        Serial.println("endPacket() failed.");
        return false;
    }

    return true;
}

void receiveFrames() {
    int packetSize = CAN.parsePacket();

    if (packetSize <= 0) return;
    if (CAN.packetRtr()) return; // Ignore RTRs.

    u32 id = CAN.packetId();
    u8 data[8] = { 0 };
    int dataLength = 0;

    while (dataLength < packetSize && dataLength < sizeof(data)) {
        int byteRead = CAN.read();
        if (byteRead == -1) break;
        data[dataLength++] = byteRead;
    }

    if (id != 0x750 || dataLength < 1 || data[0] != 0x2a) return;

    if (dataLength >= 3 && data[1] == 0x30 && data[2] == 0x00) {
        fakeTpmsEcu.handleFrameAck(data[3]);
        return;
    }

    if (dataLength < 3 || data[1] != 0x02 || data[2] != 0x21) return;

    if (data[3] == 0x30) {
        // TPMS pressures request.
        u8 response[8] = { 0 };
        response[0] = 0x2a;
        response[1] = 0x10;  // "1" means "first frame in a sequence"
        response[2] = 0x07;
        response[3] = 0x61;
        response[4] = 0x30;
        response[5] = 0xAB;  // FL tire pressure
        response[6] = 0xAC;  // FR tire pressure
        response[7] = 0xAD;  // RR tire pressure
        sendFrame(0x758, response, 8);

        response[0] = 0x2a;
        response[1] = 0x21;  // "2" means "continuation frame", "1" means "first continuation frame".
        response[2] = 0xAE;  // RL tire pressure
        response[3] = 0x00;
        response[4] = 0x00;
        response[5] = 0x00;
        response[6] = 0x00;
        response[7] = 0x00;
        fakeTpmsEcu.scheduleNextFrame(0x758, response, 8);
    }
    else if (data[3] == 0x16) {
        // TPMS temperatures request.
        u8 response[8] = { 0 };
        response[0] = 0x2a;
        response[1] = 0x10;  // "1" means "first frame in a sequence"
        response[2] = 0x07;
        response[3] = 0x61;
        response[4] = 0x16;
        response[5] = 40 + 21;  // FL tire temperature: 21ºC
        response[6] = 40 + 22;  // FR tire temperature
        response[7] = 40 + 23;  // RR tire temperature
        sendFrame(0x758, response, 8);

        response[0] = 0x2a;
        response[1] = 0x21;  // "2" means "continuation frame", "1" means "first continuation frame".
        response[2] = 40 + 24;  // RL tire temperature
        response[3] = 0x00;
        response[4] = 0x00;
        response[5] = 0x00;
        response[6] = 0x00;
        response[7] = 0x00;
        fakeTpmsEcu.scheduleNextFrame(0x758, response, 8);
    }

}

void generatePayload(u16 pid, u8* payload) {
    memset(payload, /* value= */ 0, /* size= */ 8);

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
    }
}