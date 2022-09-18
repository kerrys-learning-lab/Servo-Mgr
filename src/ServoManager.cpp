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
#include "ServoManager.hpp"
#include <memory>
#include <string>
#include "i2c_pwm/Pca9685.hpp"
#include "PwmServo.hpp"

namespace
{
  const uint8_t MAX_BOARDS = 62;
} // namespace

ServoManager::ServoManager()
    : pcaBoards_(MAX_BOARDS), logger_(rclcpp::get_logger("ServoManager"))
{
}

ServoManager::~ServoManager()
{
}

void ServoManager::configurePca9685(uint8_t id,
                                    const std::string &deviceFile,
                                    int address,
                                    bool autoInitialize)
{
  if (pcaBoards_[id])
  {
    std::ostringstream ostr;
    ostr << "PCA9685 board " << id << " already exists";
    throw std::runtime_error(ostr.str());
  }

  std::shared_ptr<i2c_pwm::Pca9685> pcaBoard = i2c_pwm::Pca9685::create(deviceFile,
                                                                        address,
                                                                        autoInitialize);
  pcaBoards_[id] = pcaBoard;

  RCLCPP_INFO(logger_, "Created PCA9685 board at slot %d", id);
}

void ServoManager::configureServo(uint16_t id,
                                  uint16_t center,
                                  uint16_t range,
                                  bool invertDirection,
                                  uint16_t defaultValue)
{
  const uint8_t boardId = id / i2c_pwm::Pca9685::NUM_CHANNELS;

  std::shared_ptr<i2c_pwm::Pca9685> pcaBoard = pcaBoards_[boardId];
  if (!pcaBoard)
  {
    std::ostringstream ostr;
    ostr << "PCA9685 board " << boardId << " does not exist";
    throw std::runtime_error(ostr.str());
  }

  servo_collection::const_iterator findIter = servos_.find(id);
  std::shared_ptr<PwmServo> servo;

  if (findIter != servos_.end())
  {
    servo = findIter->second;
  }
  else
  {
    servo = std::shared_ptr<PwmServo>(new PwmServo(pcaBoard, id));
    servos_[id] = servo;

    RCLCPP_INFO(logger_,
                "Created Servo %d (Board: %d, Channel: %d)",
                id,
                boardId,
                id % i2c_pwm::Pca9685::NUM_CHANNELS);
  }

  servo->configure(center, range, invertDirection, defaultValue);
}

void ServoManager::setAll(uint16_t value)
{
  for (auto iter = servos_.begin(); iter != servos_.end(); ++iter)
  {
    iter->second->set(value);
  }
}

void ServoManager::setAll(float value)
{
  for (auto iter = servos_.begin(); iter != servos_.end(); ++iter)
  {
    iter->second->set(value);
  }
}

void ServoManager::resetServo(uint16_t id)
{
  servo_collection::const_iterator iter = servos_.find(id);

  if (iter != servos_.end())
  {
    iter->second->reset();
  }
  else
  {
    std::ostringstream ostr;
    ostr << "Servo " << id << " does not exist";
    throw std::runtime_error(ostr.str());
  }
}

void ServoManager::setServo(uint16_t id, uint16_t data)
{
  servo_collection::const_iterator iter = servos_.find(id);

  if (iter != servos_.end())
  {
    iter->second->set(data);
  }
  else
  {
    std::ostringstream ostr;
    ostr << "Servo " << id << " does not exist";
    throw std::runtime_error(ostr.str());
  }
}

void ServoManager::setServo(uint16_t id, float data)
{
  servo_collection::const_iterator iter = servos_.find(id);

  if (iter != servos_.end())
  {
    iter->second->set(data);
  }
  else
  {
    std::ostringstream ostr;
    ostr << "Servo " << id << " does not exist";
    throw std::runtime_error(ostr.str());
  }
}
