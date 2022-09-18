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
#ifndef SERVOMANAGERNODE_HPP_
#define SERVOMANAGERNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "servo_mgr/msg/servo_control.hpp"
#include "servo_mgr/srv/configure_pca9685.hpp"
#include "servo_mgr/srv/configure_servo.hpp"
#include "servo_mgr/srv/test_servo.hpp"
#include "servo_mgr/ServoManagerVisibilityControl.hpp"

class ServoManager;

namespace servo_mgr
{

  class ServoManagerNode : public rclcpp::Node
  {
  public:
    SERVO_MGR_PUBLIC
    explicit ServoManagerNode(const rclcpp::NodeOptions &options);
    virtual ~ServoManagerNode();

  private:
    void onConfigurePCA9685(const servo_mgr::srv::ConfigurePCA9685::Request::SharedPtr request,
                            servo_mgr::srv::ConfigurePCA9685::Response::SharedPtr response);

    void onConfigureServo(const servo_mgr::srv::ConfigureServo::Request::SharedPtr request,
                          servo_mgr::srv::ConfigureServo::Response::SharedPtr response);

    void onTestServo(const servo_mgr::srv::TestServo::Request::SharedPtr request,
                     servo_mgr::srv::TestServo::Response::SharedPtr response);

    void onServoControl(const servo_mgr::msg::ServoControl::SharedPtr msg);
    void onServoControlAbsolute(const servo_mgr::msg::ServoControl::SharedPtr msg);

    rclcpp::Service<servo_mgr::srv::ConfigurePCA9685>::SharedPtr configurePca9685Svc_;
    rclcpp::Service<servo_mgr::srv::ConfigureServo>::SharedPtr configureServoSvc_;
    rclcpp::Service<servo_mgr::srv::TestServo>::SharedPtr testServoSvc_;
    rclcpp::Subscription<servo_mgr::msg::ServoControl>::SharedPtr servoControlSubscriber_;
    rclcpp::Subscription<servo_mgr::msg::ServoControl>::SharedPtr servoControlAbsoluteSubscriber_;
    std::unique_ptr<ServoManager> impl_;
  };

} // namespace servo_mgr

#endif // SERVOMANAGERNODE_HPP_
