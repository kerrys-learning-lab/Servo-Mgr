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
#ifndef PWMSERVO_HPP_
#define PWMSERVO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <memory>
#include <string>

namespace i2c_pwm
{
  class Pca9685;
}

class PwmServo
{
public:
  /**
   * @brief Construct a new Pwm Servo object
   *
   * @param pcaBoard Reference to the i2c_pwm::Pca9685 board which controls this
   *                  PwmServo
   * @param id Unique identifier/channel for this PwmServo
   */
  PwmServo(std::shared_ptr<i2c_pwm::Pca9685> pcaBoard, uint16_t id);
  virtual ~PwmServo();

  /**
   * @brief Returns the identifier/channel for this PwmServo
   *
   * @return uint16_t
   */
  uint16_t id() const;

  /**
   * @brief Returns the center value for this PwmServo.  Defaults to
   * i2c_pwm::Pca9685::MAX_VALUE / 2.  May be adjusted using PwmServo::configure
   *
   * @return uint16_t
   */
  uint16_t center() const;

  /**
   * @brief Returns the range value for this PwmServo.  Defaults to
   * i2c_pwm::Pca9685::MAX_VALUE.  May be adjusted using PwmServo::configure
   *
   * @return uint16_t
   */
  uint16_t range() const;

  bool invertDirection() const;

  /**
   * @brief Returns the current value for this PwmServo
   *
   * @return uint16_t
   */
  uint16_t value() const;

  /**
   * @brief Configures the center, range, and default values for this PwmServo,
   * as well as whether the servo direction is inverted.
   *
   * @param center
   * @param range
   * @param invertDirection
   * @param defaultValue
   */
  void configure(uint16_t center,
                 uint16_t range,
                 bool invertDirection,
                 uint16_t defaultValue);

  /**
   * @brief Resets the servo to its default value
   */
  void reset();

  /**
   * @brief Sets the absolute value of the servo in the interval [0, 4096]
   *
   * When working with a continuous rotation servo, sets the speed of the servo.
   *
   * When working with a fixed 180 degree rotation servo, sets the angle of
   * the servo.
   *
   * Hint: setting the servo pulse value to zero (0) causes the servo to power
   * off. This is referred to as 'coast'. setting a servo to its center value
   * leaves the servo powered and is referred to as 'brake'.
   *
   * @param value the absolute value of the servo, in the interval [0, 4096]
   */
  void set(uint16_t value);

  /**
   * @brief Sets the proportional value of the servo in the interval [-1.0, 1.0]
   *
   * When working with a continuous rotation servo, sets the speed of the servo.
   *
   * When working with a fixed 180 degree rotation servo, sets the angle of
   * the servo.
   *
   * @param value the proportional value of the servo, in the interval [-1.0, 1.0]
   */
  void set(float value);

private:
  template <typename T>
  T clamp(T value,
          T min,
          T max,
          const std::string &msg) const;

  std::shared_ptr<i2c_pwm::Pca9685> pcaBoard_;
  const uint16_t id_;
  uint16_t center_;
  uint16_t range_;
  bool invertDirection_;
  uint16_t defaultValue_;
  uint16_t value_;
  rclcpp::Logger logger_;
};

// ============================================================================

template <typename T>
T PwmServo::clamp(T value, T min, T max, const std::string &msg) const
{
  const T newValue = std::min(std::max(value, min), max);

  if (newValue != value)
  {
    RCLCPP_WARN(logger_,
                "[Servo %d] Invalid %s value: %s (must be in the interval [%s, %s])",
                id_,
                msg.c_str(),
                std::to_string(value).c_str(),
                std::to_string(min).c_str(),
                std::to_string(max).c_str());
  }

  return newValue;
}
#endif // PWMSERVO_HPP_
