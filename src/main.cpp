#include <Arduino.h>
#include <FreeRTOS.h>
#include <WiFi.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <string_view>
#include <utility>

WiFiClient client;

#define SSTR(s) (#s)
#define STR(s) SSTR(s)

class PingPonger {
 public:
  void Begin(IPAddress addr) {
    client.setSync(true);
    if (!client.connect(addr, 51234)) {
      Serial1.println("Failed to connect.");
      configASSERT(false);
    }

    parent_task_ = xTaskGetCurrentTaskHandle();
    xTaskCreate(
        [](void* arg) {
          // Run only on core 0. This may cause rescheduling.
          vTaskCoreAffinitySet(xTaskGetCurrentTaskHandle(), 0b1);
          reinterpret_cast<PingPonger*>(arg)->DoPingPong();
        },
        "PingPonger", 256, this, 1, nullptr);
    xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
    Serial1.println("First ping pong done");
  }

  void DoPingPong() {
    while (true) {
      client.write("ping", 4);

      uint8_t buf[4];
      int recvd = 0;

      while (recvd < 4) {
        recvd += client.read(&buf[recvd], 4 - recvd);
      }

      configASSERT(recvd == 4);
      configASSERT(std::string_view(reinterpret_cast<char*>(buf), 4) == "pong");
      xTaskNotifyGive(parent_task_);
      delay(100);
    }
  }

  TaskHandle_t parent_task_;
};

void setup_pingpong() {
  auto* ping_ponger = new PingPonger;
  Serial1.println("ping_ponger.begin");
  ping_ponger->Begin(IPAddress(192, 168, 4, 22));
  Serial1.println("ping_ponger active");
}

void setup_interrupt() {
  pinMode(15, INPUT_PULLUP);
  attachInterrupt(15, []() { Serial1.println("Interrupted!"); }, FALLING);
  Serial1.println("set up interrupt");
}

void setup() {
  Serial1.begin();
  Serial1.println("Wifi.begin");
  WiFi.begin(STR(WIFI_SSID), STR(WIFI_PASSWORD));
  while (!WiFi.isConnected()) {
    delay(100);
  }
  Serial1.println("Wifi ready");

  Serial1.println("Setting up test.");
#if INTERRUPT_SECOND != 0
  setup_pingpong();
  setup_interrupt();
#else
  setup_interrupt();
  setup_pingpong();
#endif
  pinMode(14, OUTPUT_4MA);
  digitalWrite(14, HIGH);
};

bool on = false;

void loop() {
  delay(1000);
  if (!on) {
    Serial1.println("Sending signal that should cause interrupt . . .");
  }

  digitalWrite(14, on);
  on = !on;
}