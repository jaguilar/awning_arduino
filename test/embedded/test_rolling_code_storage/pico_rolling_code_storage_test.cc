#include "pico_rolling_code_storage.h"

#include <Arduino.h>
#include <unity.h>

#include <cstdio>

using awning::PicoFlashRCS;

void test_CreateAndDestroyRCS() {
  PicoFlashRCS rcs;
  rcs.Reset(0);
  TEST_ASSERT_EQUAL_INT(0, rcs.nextCode());
  TEST_ASSERT_EQUAL_INT(1, rcs.nextCode());
  TEST_ASSERT_EQUAL_INT(2, rcs.nextCode());
  rcs.Reset(1234);
  TEST_ASSERT_EQUAL_INT(1234, rcs.nextCode());
}

void test_RecreateHasNextCode() {
  uint16_t first_code, second_code;
  {
    PicoFlashRCS rcs;
    first_code = rcs.nextCode();
  }
  {
    PicoFlashRCS rcs;
    second_code = rcs.nextCode();
  }

  TEST_ASSERT_EQUAL_INT(first_code + 1, second_code);
}

void test_WrapAround() {
  {
    PicoFlashRCS rcs;
    rcs.Reset(std::numeric_limits<uint16_t>::max() - 1);
    TEST_ASSERT_EQUAL_INT(std::numeric_limits<uint16_t>::max() - 1,
                          rcs.nextCode());
    TEST_ASSERT_EQUAL_INT(std::numeric_limits<uint16_t>::max(), rcs.nextCode());
    TEST_ASSERT_EQUAL_INT(0, rcs.nextCode());
  }

  {
    // Verify that the flash was indeed reprogrammed and we didn't merely
    // update a memory value.
    PicoFlashRCS rcs;
    TEST_ASSERT_EQUAL_INT(1, rcs.nextCode());
  }
}

void tearDown() {}

void setUp() {}

int runUnityTests(void) {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_CreateAndDestroyRCS);
  RUN_TEST(test_RecreateHasNextCode);
  RUN_TEST(test_WrapAround);
  return UNITY_END();
}

void setup() {
  Serial1.begin();
  runUnityTests();
}

void loop() {}