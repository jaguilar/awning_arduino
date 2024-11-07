#include "unity_config.h"

#include <Arduino.h>

extern "C" {

void serial1_write(char c) { Serial1.write(c); }

}