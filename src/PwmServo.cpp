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
#include "PwmServo.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <memory>
#include "i2c_pwm/Pca9685.hpp"

PwmServo::PwmServo(std::shared_ptr<i2c_pwm::Pca9685> pcaBoard, uint16_t id)
    : pcaBoard_(pcaBoard),
      id_(id),
      center_(i2c_pwm::Pca9685::MAX_VALUE / 2),
      range_(i2c_pwm::Pca9685::MAX_VALUE),
      invertDirection_(false),
      defaultValue_(i2c_pwm::Pca9685::MAX_VALUE / 2),
      value_(0),
      logger_(rclcpp::get_logger("PwmServo"))
{
}

PwmServo::~PwmServo()
{
}

uint16_t PwmServo::id() const
{
  return id_;
}

uint16_t PwmServo::center() const
{
  return center_;
}

uint16_t PwmServo::range() const
{
  return range_;
}

bool PwmServo::invertDirection() const
{
  return invertDirection_;
}

uint16_t PwmServo::value() const
{
  return value_;
}

void PwmServo::configure(uint16_t center,
                         uint16_t range,
                         bool invertDirection,
                         uint16_t defaultValue)
{
  invertDirection_ = invertDirection;

  center_ = clamp(center,
                  i2c_pwm::Pca9685::MIN_VALUE,
                  i2c_pwm::Pca9685::MAX_VALUE,
                  "center");

  range_ = clamp(range,
                 i2c_pwm::Pca9685::MIN_VALUE,
                 i2c_pwm::Pca9685::MAX_VALUE,
                 "range");

  const float halfRange = range_ / 2.0;
  if (center_ - halfRange < i2c_pwm::Pca9685::MIN_VALUE || halfRange + center_ > i2c_pwm::Pca9685::MAX_VALUE)
  {
    RCLCPP_WARN(logger_,
                "[Servo %d] Invalid range/center combination %d/%d (must be in the interval [0, %d])",
                id_,
                center_,
                range_,
                i2c_pwm::Pca9685::MAX_VALUE);

    center_ = range_ / 2;
  }

  defaultValue_ = clamp(defaultValue,
                        i2c_pwm::Pca9685::MIN_VALUE,
                        i2c_pwm::Pca9685::MAX_VALUE,
                        "defaultValue");

  RCLCPP_INFO(logger_,
              "[Servo %d] Configured: center=%d, range=%d, direction=%d (default=%d)",
              id_,
              center_,
              range_,
              invertDirection_ ? -1 : 1,
              defaultValue_);
}

void PwmServo::reset()
{
  set(defaultValue_);
}

void PwmServo::set(uint16_t value)
{
  const uint16_t clampedValue = std::min(value, i2c_pwm::Pca9685::MAX_VALUE);
  if (value > i2c_pwm::Pca9685::MAX_VALUE)
  {
    RCLCPP_WARN(logger_,
                "[Servo %d] Clamped value %d to %d instead",
                id_,
                value,
                clampedValue);
  }

  value_ = clampedValue;
  RCLCPP_DEBUG(logger_, "[Servo %d] Set value to %d", id_, value_);

  pcaBoard_->writeChannel(id_ % i2c_pwm::Pca9685::NUM_CHANNELS, value_);
}

void PwmServo::set(float value)
{
  const float clampedValue = std::max(std::min(value, 1.0f), -1.0f);

  if (clampedValue != value)
  {
    RCLCPP_WARN(logger_,
                "[Servo %d] Clamped value %f to %f instead",
                id_,
                value,
                clampedValue);
  }

  const float halfRange = range_ / 2.0;
  const uint16_t pos = ((invertDirection_ ? -1 : 1) * (halfRange * clampedValue)) + center_;

  RCLCPP_DEBUG(logger_,
               "[Servo %d] Converted proportional value to absolute: %f --> %d",
               id_,
               value,
               pos);

  set(pos);
}
