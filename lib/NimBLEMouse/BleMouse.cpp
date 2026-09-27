#include "BleMouse.h"

#include <NimBLEDevice.h>
#include "HIDTypes.h"

static const uint8_t hidReportDescriptor[] = {
    USAGE_PAGE(1), 0x01, USAGE(1), 0x02, COLLECTION(1), 0x01,
    USAGE(1), 0x01, COLLECTION(1), 0x00,
    USAGE_PAGE(1), 0x09, USAGE_MINIMUM(1), 0x01, USAGE_MAXIMUM(1), 0x05,
    LOGICAL_MINIMUM(1), 0x00, LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1), 0x01, REPORT_COUNT(1), 0x05, HIDINPUT(1), 0x02,
    REPORT_SIZE(1), 0x03, REPORT_COUNT(1), 0x01, HIDINPUT(1), 0x03,
    USAGE_PAGE(1), 0x01, USAGE(1), 0x30, USAGE(1), 0x31, USAGE(1), 0x38,
    LOGICAL_MINIMUM(1), 0x81, LOGICAL_MAXIMUM(1), 0x7f,
    REPORT_SIZE(1), 0x08, REPORT_COUNT(1), 0x03, HIDINPUT(1), 0x06,
    USAGE_PAGE(1), 0x0c, USAGE(2), 0x38, 0x02,
    LOGICAL_MINIMUM(1), 0x81, LOGICAL_MAXIMUM(1), 0x7f,
    REPORT_SIZE(1), 0x08, REPORT_COUNT(1), 0x01, HIDINPUT(1), 0x06,
    END_COLLECTION(0), END_COLLECTION(0)
};

BleMouse::BleMouse(std::string deviceName, std::string deviceManufacturer, uint8_t batteryLevel)
    : batteryLevel(batteryLevel), deviceManufacturer(deviceManufacturer), deviceName(deviceName) {}

void BleMouse::begin() {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setSecurityAuth(true, false, true);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(this);
    hid = new NimBLEHIDDevice(server);
    inputMouse = hid->getInputReport(0);
    hid->setManufacturer(deviceManufacturer);
    hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->setHidInfo(0x00, 0x02);
    hid->setReportMap(const_cast<uint8_t*>(hidReportDescriptor), sizeof(hidReportDescriptor));
    hid->startServices();
    NimBLEAdvertising* advertising = server->getAdvertising();
    advertising->setAppearance(HID_MOUSE);
    advertising->addServiceUUID(hid->getHidService()->getUUID());
    advertising->start();
    hid->setBatteryLevel(batteryLevel);
}

void BleMouse::end() {}

void BleMouse::click(uint8_t b) {
    _buttons = b;
    move(0, 0, 0, 0);
    _buttons = 0;
    move(0, 0, 0, 0);
}

void BleMouse::move(signed char x, signed char y, signed char wheel, signed char hWheel) {
    if (!isConnected() || inputMouse == nullptr) return;
    uint8_t report[5] = {_buttons, static_cast<uint8_t>(x), static_cast<uint8_t>(y), static_cast<uint8_t>(wheel), static_cast<uint8_t>(hWheel)};
    inputMouse->setValue(report, sizeof(report));
    inputMouse->notify();
}

void BleMouse::buttons(uint8_t buttons) {
    if (buttons == _buttons) return;
    _buttons = buttons;
    move(0, 0, 0, 0);
}

void BleMouse::press(uint8_t button) { buttons(_buttons | button); }
void BleMouse::release(uint8_t button) { buttons(_buttons & ~button); }
bool BleMouse::isPressed(uint8_t button) const { return (_buttons & button) != 0; }
bool BleMouse::isConnected() const { return connected; }

void BleMouse::setBatteryLevel(uint8_t level) {
    batteryLevel = level;
    if (hid != nullptr) hid->setBatteryLevel(level);
}

void BleMouse::onConnect(NimBLEServer*, NimBLEConnInfo&) {
    connected = true;
    if (connectCallback) connectCallback();
}

void BleMouse::onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) {
    connected = false;
    if (disconnectCallback) disconnectCallback();
}

void BleMouse::onConnect(Callback callback) { connectCallback = callback; }
void BleMouse::onDisconnect(Callback callback) { disconnectCallback = callback; }
