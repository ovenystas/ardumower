#include "gtest/gtest.h"
#include "arduino-mock/Arduino.h"

#include "../../ardumower/pid.cpp"

using ::testing::Return;

TEST(pidTest, constructor) {
  PID pid = PID(1.1, 2.2, 3.3);
  EXPECT_FLOAT_EQ(1.1, pid.Kp);
  EXPECT_FLOAT_EQ(2.2, pid.Ki);
  EXPECT_FLOAT_EQ(3.3, pid.Kd);
}

TEST(pidTest, compute) {
  ArduinoMock* arduinoMock = arduinoMockInstance();
  PID pid = PID(1.0, 1.0, 1.0);
  EXPECT_FLOAT_EQ(1.0, pid.Kp);
  EXPECT_FLOAT_EQ(1.0, pid.Ki);
  EXPECT_FLOAT_EQ(1.0, pid.Kd);
  EXPECT_CALL(*arduinoMock, micros()).WillOnce(Return(100000.0));
  pid.x = 0.0;
  pid.w = 1.0;
  pid.y_min = -1000.0;
  pid.y_max = 1000.0;
  pid.max_output = 1000.0;
  pid.lastControlTime = 0.0;
  pid.esum = 0.0;
  pid.eold = 0.0;
  float y = pid.compute();
  EXPECT_FLOAT_EQ(11.1, y);
  EXPECT_FLOAT_EQ(1.0, pid.eold);
  EXPECT_FLOAT_EQ(1.0, pid.esum);
  EXPECT_FLOAT_EQ(0.1, pid.Ta);
  EXPECT_FLOAT_EQ(100000.0, pid.lastControlTime);
  releaseArduinoMock();
}
