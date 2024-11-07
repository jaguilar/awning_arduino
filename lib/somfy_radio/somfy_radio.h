#ifndef SOMFY_RADIO_H

#include <Arduino.h>

#include <memory>

namespace awning {

// Combines a SomfyRemote and the radio it actuates. This class hides its
// members because the somfy radio library introduces a type called Command
// into the global namespace come on who does that.
class SomfyRadio {
 public:
  SomfyRadio();
  ~SomfyRadio();

  void begin();

  enum Command {
    My = 0x1,
    Up = 0x2,
    MyUp = 0x3,
    Down = 0x4,
    MyDown = 0x5,
    UpDown = 0x6,
    Prog = 0x8,
    SunFlag = 0x9,
    Flag = 0xA
  };
  void transmit(Command c);

 private:
  struct State;

  std::unique_ptr<State> state_;
};

}  // namespace awning

#endif