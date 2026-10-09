#include "ApdsSensor.h"

namespace {
constexpr uint8_t APDS_ENABLE_EXPECTED = 0x4D;
}

bool ApdsSensor::begin(uint8_t& chipId) {
    if (!sensor_.readDeviceID(chipId)) {
        Serial.println("[Sensor] Failed to read APDS9960 chip ID at 0x39");
        return false;
    }
    if (chipId != APDS9960_ID_1 && chipId != APDS9960_ID_2) {
        Serial.printf("[Sensor] Unsupported chip ID 0x%02X; expected 0xAB or 0x9C\n",
                      chipId);
        return false;
    }
    if (!sensor_.init()) {
        Serial.println("[Sensor] SparkFun APDS9960 initialization failed");
        return false;
    }
    if (!sensor_.enableGestureSensor(false)) {
        Serial.println("[Sensor] SparkFun gesture engine initialization failed");
        return false;
    }

    const uint8_t enable = sensor_.getMode();
    const uint8_t gestureMode = sensor_.getGestureMode();
    if (enable != APDS_ENABLE_EXPECTED || gestureMode != 0x01) {
        Serial.printf("[Sensor] Startup verification failed: ENABLE=0x%02X GCONF4=0x%02X\n",
                      enable, gestureMode);
        return false;
    }

    Serial.printf("[Sensor] SparkFun APDS9960 initialized (ID=0x%02X, ENABLE=0x%02X)\n",
                  chipId, enable);
    return true;
}

bool ApdsSensor::readProximity(uint8_t& proximity) {
    static unsigned long lastErrorMs = 0;
    if (sensor_.readProximity(proximity)) {
        return true;
    }

    if (millis() - lastErrorMs >= 1000) {
        lastErrorMs = millis();
        Serial.println("[Sensor] Failed to read proximity register");
    }
    return false;
}

ApdsGesture ApdsSensor::readGesture() {
    static unsigned long lastErrorMs = 0;
    const int gesture = sensor_.readGestureNonBlocking();
    if (gesture == ERROR) {
        if (millis() - lastErrorMs >= 1000) {
            lastErrorMs = millis();
            Serial.println("[Sensor] Non-blocking gesture FIFO read failed");
        }
        return ApdsGesture::None;
    }

    switch (gesture) {
        case DIR_UP: return ApdsGesture::Up;
        case DIR_DOWN: return ApdsGesture::Down;
        case DIR_LEFT: return ApdsGesture::Left;
        case DIR_RIGHT: return ApdsGesture::Right;
        default: return ApdsGesture::None;
    }
}
