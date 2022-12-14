// Copyright 2022 Kerry Johnson
//
// This file is part of Servo-Mgr.
//
// Servo-Mgr is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License v3 as published by the Free Software
// Foundation.
//
// Servo-Mgr is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// Servo-Mgr.  If not, see https://www.gnu.org/licenses/.
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <utility>
#include "i2c_pwm/Pca9685.hpp"
#include "PwmServo.hpp"

namespace
{
  class MockPca9685 : public i2c_pwm::Pca9685
  {
  public:
    MockPca9685()
        : channel(0), onValue(0), offValue(0)
    {
    }

    virtual ~MockPca9685()
    {
    }

    void initialize() override
    {
    }

    void allStop() override
    {
    }

    void sleepMode(__attribute__((unused)) bool _) override
    {
    }

    void setFrequencyHz(uint16_t) override
    {
    }

    uint8_t read(uint8_t reg) const override
    {
      return reg;
    }

    void write(uint8_t, uint8_t) override
    {
    }

    void writeChannel(uint8_t inChannel, uint16_t off) override
    {
      writeChannel(inChannel, 0, off);
    }

    void writeChannel(uint8_t inChannel, uint16_t on, uint16_t off)
        override
    {
      this->channel = inChannel;
      this->onValue = on;
      this->offValue = off;
    }

    uint8_t channel;
    uint16_t onValue;
    uint16_t offValue;
  };
} // namespace

namespace i2c_pwm
{

  TEST(PwmServoTests, testConstructor)
  {
    const uint16_t EXPECTED_ID = 1;

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    EXPECT_EQ(uut.id(), EXPECTED_ID);
    EXPECT_EQ(uut.center(), Pca9685::MAX_VALUE / 2);
    EXPECT_EQ(uut.range(), Pca9685::MAX_VALUE);
    EXPECT_EQ(uut.invertDirection(), false);
    EXPECT_EQ(uut.value(), 0);
  }

  TEST(PwmServoTests, testConfigure)
  {
    const uint16_t EXPECTED_ID = 1;

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    uut.configure(Pca9685::MAX_VALUE / 2,
                  Pca9685::MAX_VALUE,
                  false,
                  Pca9685::MAX_VALUE / 2);
    EXPECT_EQ(uut.center(), Pca9685::MAX_VALUE / 2);
    EXPECT_EQ(uut.range(), Pca9685::MAX_VALUE);
    EXPECT_EQ(uut.invertDirection(), false);
  }

  TEST(PwmServoTests, testConfigureOutOfRange)
  {
    const uint16_t EXPECTED_ID = 1;

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    uut.configure(Pca9685::MAX_VALUE + 1,
                  Pca9685::MAX_VALUE + 1,
                  false,
                  Pca9685::MAX_VALUE / 2);
    EXPECT_EQ(uut.center(), Pca9685::MAX_VALUE / 2);
    EXPECT_EQ(uut.range(), Pca9685::MAX_VALUE);
    EXPECT_EQ(uut.invertDirection(), false);
  }

  TEST(PwmServoTests, testSetAbsolute)
  {
    const uint16_t EXPECTED_ID = 17;
    const uint16_t EXPECTED_CHANNEL = 1; // 17 % 16
    const uint16_t VALUE = 4;

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    uut.set(VALUE);

    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->offValue, VALUE);
  }

  TEST(PwmServoTests, testSetAbsoluteInvalid)
  {
    const uint16_t EXPECTED_ID = 17;
    const uint16_t EXPECTED_CHANNEL = 1; // 17 % 16
    const uint16_t VALUE = Pca9685::MAX_VALUE + 42;

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    uut.set(VALUE);

    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->offValue, Pca9685::MAX_VALUE);
  }

  TEST(PwmServoTests, testSetProportional)
  {
    //  |-----------|-----------|
    // Min        Center       Max
    //               <---------->
    //                Half-range
    const uint16_t HALF_RANGE = Pca9685::MAX_VALUE / 2;
    const float STEP_SIZE_PROPORTIONAL = 0.25f;
    const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

    const uint16_t EXPECTED_ID = 35;
    const uint16_t EXPECTED_CHANNEL = 3; // 35 % 16
    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    for (int i = 0; i <= Pca9685::MAX_VALUE / STEP_SIZE_ABSOLUTE; ++i)
    {
      uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
      EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
      EXPECT_EQ(mockPca9685->offValue,
                Pca9685::MIN_VALUE + (i * STEP_SIZE_ABSOLUTE))
          << "Step: " << i;
    }
  }

  TEST(PwmServoTests, testSetProportionalCustomRange)
  {
    //  |-----------|-----------|
    // Min        Center       Max
    //               <---------->
    //                Half-range
    const uint16_t CUSTOM_RANGE = 1000;
    const uint16_t HALF_RANGE = CUSTOM_RANGE / 2;
    const float STEP_SIZE_PROPORTIONAL = 0.25f;
    const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

    const uint16_t EXPECTED_ID = 35;
    const uint16_t EXPECTED_CHANNEL = 3; // 35 % 16
    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);
    uut.configure(CUSTOM_RANGE / 2, CUSTOM_RANGE, false, CUSTOM_RANGE / 2);

    for (int i = 0; i <= CUSTOM_RANGE / STEP_SIZE_ABSOLUTE; ++i)
    {
      uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
      EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
      EXPECT_EQ(mockPca9685->offValue,
                Pca9685::MIN_VALUE + (i * STEP_SIZE_ABSOLUTE))
          << "Step: " << i;
    }
  }

  TEST(PwmServoTests, testSetProportionalInverted)
  {
    //  |-----------|-----------|
    // Min        Center       Max
    //               <---------->
    //                Half-range
    const uint16_t HALF_RANGE = Pca9685::MAX_VALUE / 2;
    const float STEP_SIZE_PROPORTIONAL = 0.25f;
    const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

    const uint16_t EXPECTED_ID = 35;
    const uint16_t EXPECTED_CHANNEL = 3; // 35 % 16

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);
    uut.configure(Pca9685::MAX_VALUE / 2,
                  Pca9685::MAX_VALUE,
                  true,
                  Pca9685::MAX_VALUE / 2);

    for (int i = 0; i <= Pca9685::MAX_VALUE / STEP_SIZE_ABSOLUTE; ++i)
    {
      uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
      EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
      EXPECT_EQ(mockPca9685->offValue,
                Pca9685::MAX_VALUE - (i * STEP_SIZE_ABSOLUTE))
          << "Step: " << i;
    }
  }

  TEST(PwmServoTests, testSetProportionalInvertedCustomRange)
  {
    //  |-----------|-----------|
    // Min        Center       Max
    //               <---------->
    //                Half-range
    const uint16_t CUSTOM_RANGE = 1000;
    const uint16_t HALF_RANGE = CUSTOM_RANGE / 2;
    const float STEP_SIZE_PROPORTIONAL = 0.25f;
    const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

    const uint16_t EXPECTED_ID = 35;
    const uint16_t EXPECTED_CHANNEL = 3; // 35 % 16
    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);
    uut.configure(CUSTOM_RANGE / 2, CUSTOM_RANGE, true, CUSTOM_RANGE / 2);

    for (int i = 0; i <= CUSTOM_RANGE / STEP_SIZE_ABSOLUTE; ++i)
    {
      uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
      EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
      EXPECT_EQ(mockPca9685->offValue,
                CUSTOM_RANGE - (i * STEP_SIZE_ABSOLUTE))
          << "Step: " << i;
    }
  }

  TEST(PwmServoTests, testSetProportionalInvalid)
  {
    const uint16_t EXPECTED_ID = 17;
    const uint16_t EXPECTED_CHANNEL = 1; // 17 % 16
    const uint16_t VALUE = Pca9685::MAX_VALUE + 42;

    std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

    PwmServo uut(mockPca9685, EXPECTED_ID);

    uut.set(VALUE);

    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->offValue, Pca9685::MAX_VALUE);
  }

} // namespace i2c_pwm

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
