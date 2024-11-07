
#include <Arduino.h>
#include <FreeRTOS.h>
#include <HADevice.h>
#include <HAMQTT.h>
#include <WiFi.h>
#include <device-types/HACover.h>
#include <queue.H>
#include <somfy_radio.h>
#include <task.h>

using namespace awning;

class DebouncedButton {
 public:
  DebouncedButton(uint8_t pin, std::function<void()> handler)
      : pin_(pin), handler_(handler) {
    pinMode(pin_, INPUT_PULLUP);
    attachInterruptParam(pin_, &DebouncedButton::Press, FALLING, this);
    last_press_ = millis() - 5'000;
  }

  ~DebouncedButton() { detachInterrupt(pin_); }

  static void Press(void* arg) {
    auto& pin = *reinterpret_cast<DebouncedButton*>(arg);
    const int32_t now = millis();

    if (now - pin.last_press_ < 250) {
      // Only accept one press per 250ms
      return;
    }
    pin.last_press_ = now;
    pin.handler_();
  }

  uint8_t pin_;
  int32_t last_press_;
  std::function<void()> handler_;
};

SomfyRadio somfy;

WiFiClient wifi_client;
HADevice device("autoawning");
HACover cover("awning");
HAMqtt mqtt(wifi_client, device);

QueueHandle_t command_queue;

// The HACover class doesn't define My, but we need that for our chassis
// buttons, so we have our own enum.
enum CoverCommandOrMy {
  CommandClose,
  CommandOpen,
  CommandStop,
  CommandMy,
};

void DebouncedButtonHandler(CoverCommandOrMy c) {
  BaseType_t higher_priority_task_woken;
  HACover::CoverCommand command = HACover::CommandOpen;
  xQueueSendFromISR(command_queue, &c, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

DebouncedButton up_button(D20, [] { DebouncedButtonHandler(CommandOpen); });
DebouncedButton my_button(D19, [] { DebouncedButtonHandler(CommandMy); });
DebouncedButton dn_button(D18, [] { DebouncedButtonHandler(CommandClose); });

void StopCover() {
  const HACover::CoverState state = cover.getCurrentState();
  if (state == HACover::StateStopped || state == HACover::StateOpen ||
      state == HACover::StateClosed) {
    return;
  }

  if (state == HACover::StateOpening) {
    somfy.transmit(SomfyRadio::Up);
  } else {
    somfy.transmit(SomfyRadio::Down);
  }
  cover.setState(HACover::StateStopped);
}

void OpenCover() {
  const HACover::CoverState state = cover.getCurrentState();
  if (state == HACover::StateOpen || state == HACover::StateOpening) {
    return;
  }
  if (state == HACover::StateClosing) {
    StopCover();
    delay(500);
  }
  somfy.transmit(SomfyRadio::Down);
  cover.setState(HACover::StateOpening);
}

void CloseCover() {
  const HACover::CoverState state = cover.getCurrentState();
  if (state == HACover::StateClosed || state == HACover::StateClosing) {
    return;
  }
  if (state == HACover::StateOpening) {
    StopCover();
    delay(500);
  }
  somfy.transmit(SomfyRadio::Up);
  cover.setState(HACover::StateClosing);
}

void CompleteMotion() {
  const HACover::CoverState state = cover.getCurrentState();
  if (state == HACover::StateOpening) {
    cover.setState(HACover::StateOpen);
  } else if (state == HACover::StateClosing) {
    cover.setState(HACover::StateClosed);
  }
}

bool IsInMotion() {
  const HACover::CoverState state = cover.getCurrentState();
  return state == HACover::StateOpening || state == HACover::StateClosing;
}

#define STR(x) #x
#define SSTR(x) STR(x)

void setup() {
  delay(500);
  WiFi.begin(SSTR(WIFI_SSID), SSTR(WIFI_PASSWORD));

  command_queue = xQueueCreate(1, sizeof(HACover::CoverCommand));

  device.enableExtendedUniqueIds();
  device.enableLastWill();
  device.enableSharedAvailability();
  device.setName("Auto Awning");
  cover.onCommand([](HACover::CoverCommand in, HACover* sender) {
    CoverCommandOrMy out;
    switch (in) {
      case HACover::CommandStop:
        out = CommandStop;
        break;
      case HACover::CommandOpen:
        out = CommandOpen;
        break;
      case HACover::CommandClose:
        out = CommandClose;
        break;
    }

    xQueueSend(command_queue, &out, 0);
  });

  somfy.begin();

  while (WiFi.isConnected()) {
    Serial1.println("Waiting for wifi...");
    delay(1000);
  }
  Serial1.println("WiFi up");

  mqtt.begin(SSTR(MQTT_SERVER), SSTR(MQTT_USER), SSTR(MQTT_PASSWORD));
  xTaskCreate(
      [](void*) {
        while (true) {
          mqtt.loop();
          delay(5);
        }
      },
      "mqtt_loop", 2048, NULL, 1, NULL);

  while (!mqtt.isConnected()) {
    delay(1000);
  }
  Serial1.println("MQTT up");
}

void loop() {
  HACover::CoverCommand command;
  TickType_t delay = portMAX_DELAY;
  while (true) {
    if (!xQueueReceive(command_queue, &command, delay)) {
      // We timed out, which will only happen if we're in motion or a really
      // long time passes. If there's nothing to be done, CompleteMotion() is a
      // noop anyway.
      CompleteMotion();
      delay = portMAX_DELAY;
      continue;
    };

    switch (command) {
      case CommandClose:
        CloseCover();
        break;
      case CommandOpen:
        OpenCover();
        break;
      case CommandStop:
        StopCover();
        break;
      case CommandMy:
        // If we're in motion, then we stop. Otherwise we send the My command,
        // which sends my and sets the state to the opposite of its current.
        if (IsInMotion()) {
          StopCover();
        } else {
          somfy.transmit(SomfyRadio::My);
          cover.setState(cover.getCurrentState() == HACover::StateOpen
                             ? HACover::StateClosing
                             : HACover::StateOpening);
        }
    }

    // After five seconds, complete whatever motion we started.
    if (IsInMotion()) {
      delay = pdMS_TO_TICKS(5000);
    }
  }
}
