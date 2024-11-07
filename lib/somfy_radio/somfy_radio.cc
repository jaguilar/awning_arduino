#include "somfy_radio.h"

#include <RFM69.h>
#include <SPI.h>
#include <SomfyRemote.h>
#include <pico_rolling_code_storage.h>

namespace awning {

constexpr auto kSomfyRadioAddress = 0x2a3b4c;  // You'll want to customize this.
constexpr auto kDataPin = D15;

enum RfmRegister : uint8_t {
  kRfmRegisterDataModul = 0x02,
  kRfmRegisterIrqFlags = 0x27,
};

enum RfmRegisterIqrFlags : uint8_t {
  kModeReady = 0b1000'0000,
  kTxReady = 0b0010'0000,
};

struct SomfyRadio::State {
  RFM69 radio{D5, kDataPin, true, &SPI};
  awning::PicoFlashRCS rolling_code_storage;
  SomfyRemote remote{kDataPin, kSomfyRadioAddress, &rolling_code_storage};
};

SomfyRadio::SomfyRadio() : state_(std::make_unique<State>()) {}

void SomfyRadio::begin() {
  SPI.begin();
  SPI.setCS(D5);
  state_->radio.initialize(RF69_433MHZ, 1);
  state_->radio.setFrequency(433'420'000);

  // The SomfyRemote library doesn't use or provide a clock, since the protocol
  // is so slow. Therefore, use OOK in continuous mode with no sync clock.
  // Filtering requires a sync clock so we don't use that either.
  constexpr byte kModeContinuousNoSync = 0b11 << 5;
  constexpr byte kModulationOok = 0b01 << 3;
  constexpr byte kNoFiltering = 0b00;
  state_->radio.writeReg(kRfmRegisterDataModul,
                         kModeContinuousNoSync | kModulationOok | kNoFiltering);

  // Put the radio into sleep mode and wait until the radio is in ready mode
  // (i.e. it has entered sleep mode).
  state_->radio.setMode(RF69_MODE_SLEEP);
  while (!(state_->radio.readReg(kRfmRegisterIrqFlags) & kModeReady)) {
    delayMicroseconds(100);
  }

  state_->remote.setup();
}

void SomfyRadio::transmit(Command c) {
  state_->radio.setMode(RF69_MODE_TX);
  while (!(state_->radio.readReg(kRfmRegisterIrqFlags) & kTxReady)) {
    // Wait for the radio to be ready to transmit
    delayMicroseconds(100);
  }

  noInterrupts();
  state_->remote.sendCommand(static_cast<::Command>(c));
  interrupts();
}

}  // namespace awning