#ifndef ESP32_BLE_MOUSE_H
#define ESP32_BLE_MOUSE_H

#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include <functional>
#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include <NimBLEHIDDevice.h>

#define MOUSE_LEFT    1
#define MOUSE_RIGHT   2
#define MOUSE_MIDDLE  4
#define MOUSE_BACK    8
#define MOUSE_FORWARD 16
#define MOUSE_ALL     (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE)

class BleMouse : protected NimBLEServerCallbacks {
  public:
    using Callback = std::function<void(void)>;

    BleMouse(std::string deviceName = "ESP32-Mouse", std::string deviceManufacturer = "Espressif", uint8_t batteryLevel = 100);
    void begin(void);
    void end(void);
    void click(uint8_t b = MOUSE_LEFT);
    void move(signed char x, signed char y, signed char wheel = 0, signed char hWheel = 0);
    void press(uint8_t b = MOUSE_LEFT);
    void release(uint8_t b = MOUSE_LEFT);
    bool isPressed(uint8_t b = MOUSE_LEFT) const;
    bool isConnected(void) const;
    void setBatteryLevel(uint8_t level);
    void onConnect(Callback cb);
    void onDisconnect(Callback cb);

  protected:
    void buttons(uint8_t b);
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;

  protected:
    uint8_t _buttons = 0;
    NimBLEHIDDevice* hid = nullptr;
    NimBLECharacteristic* inputMouse = nullptr;
    uint8_t batteryLevel;
    std::string deviceManufacturer;
    std::string deviceName;
    bool connected = false;
    Callback connectCallback = nullptr;
    Callback disconnectCallback = nullptr;
};

#endif
#endif
