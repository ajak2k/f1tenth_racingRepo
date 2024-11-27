/*
Copyright 2024 Ajay Shankar Sriram

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

//A cpp file to hold all the utility functions, can be imported to other nodes for implimenting various functions

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

ackermann_msgs::msg::AckermannDriveStamped drive_message(float speed = 0.0, float acceleration = 0.0, float steering_angle = 0.0 )
{
    /*
    Based on the desired speed, acceleration and streeing angle create a stamped ackermann drive message.
    Sets the desired streeing angle instantly with no ramp, sets the frame_id to base link
    Args:
        speed:           desired linear velocity
        acceleration:    desired linear acceleration
        streering_angle: desired streering angle
    Returns:
        msg: the stamped ackermann drive message
    */
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "base_link";
    msg.drive.steering_angle = tan(steering_angle)/0.5 * speed;//L approximately 0.5m for the F1tenth car;
    msg.drive.steering_angle_velocity = 0.0;
    msg.drive.speed = speed;
    msg.drive.acceleration = acceleration;
    msg.drive.jerk = 0.0;
    return msg;
}