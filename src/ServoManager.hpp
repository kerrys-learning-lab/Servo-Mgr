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
#ifndef SERVOMANAGER_HPP_
#define SERVOMANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "i2c_pwm/Pca9685.hpp"

class PwmServo;

class ServoManager
{
public:
  ServoManager();
  virtual ~ServoManager();

  /**
   * @brief Sets all PwmServos managed by this ServoManager to the given value
   *
   * @param value desired value in the interval [0, 4096]
   */
  void setAll(uint16_t value);

  /**
   * @brief Sets all PwmServos managed by this ServoManager to the given
   * proportional value
   *
   * @param value desired value in the interval [-1.0, 1.0]
   */
  void setAll(float value);

  void configurePca9685(uint8_t id,
                        const std::string &deviceFile,
                        int address,
                        bool autoInitialize = true);
  void configureServo(uint16_t id,
                      uint16_t center = i2c_pwm::Pca9685::MAX_VALUE / 2,
                      uint16_t range = i2c_pwm::Pca9685::MAX_VALUE,
                      bool invertDirection = false,
                      uint16_t defaultValue = i2c_pwm::Pca9685::MAX_VALUE / 2);
  void resetServo(uint16_t id);
  void setServo(uint16_t id, uint16_t data);
  void setServo(uint16_t id, float data);

private:
  typedef std::vector<std::shared_ptr<i2c_pwm::Pca9685>> pca_board_collection;
  typedef std::map<uint16_t, std::shared_ptr<PwmServo>> servo_collection;

  pca_board_collection pcaBoards_;
  servo_collection servos_;
  rclcpp::Logger logger_;
};

#endif // SERVOMANAGER_HPP_
