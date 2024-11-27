/*
Skeleton Code based on work by Hongrui (Billy) Zheng from Upenn for ESE6150: F1Tenth Autonomous Racing Cars
https://github.com/f1tenth/f1tenth_lab2_template/tree/main

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

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "utils.hpp"

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        RCLCPP_INFO(this->get_logger(), "Constructing wall_follow_node"); 
        ///TODO: investigate Qos to set rate of sub and pub
        //Subscriptions to the car to get Lidar Data
        scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        //Publisher to the car drive topic
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    ///TODO: change the PID parameters to tunable parameters read from a params file
    // PID CONTROL PARAMS
    double kp = 3.0;
    double kd = 0.1;
    double ki = 0.0;
    double prev_error = 0.0;
    double integral_error = 0.0;
    double derivative_error = 0.0;
    double t_0 = -1.0;
    double t = 0.0;
    double t_minus_1 = 0.0;

    //Velocity limits
    double max_velocity = 1.5;
    double mid_velocity = 1.0;
    double min_velocity = 0.5;
    double acceleration = 0.0;

    //wall follow parameters
    double L = 1.5; //look ahead distance; remember car length is 0.5m
    double distance_setpoint = 1.0; //desired dist from the wall
    //assume angle of the car is = 0 if heading is straight ahead
    //all angles specified in degrees and then converted to radians, degree * 0.0174533 = radians
    double theta = 40; //angle between beams a and b
    double b_angle = 90; //angle of the beam that is along the x-axis of the car. Set to +90 to follow right wall, set to -90 to follow left wall
    double a_angle = b_angle+theta; //angle of the beam that is theta degrees ahead of the x-axis of the car
    // convert to radians
    theta *= 0.0174533;
    a_angle *= 0.0174533;
    b_angle *= 0.0174533;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    //pubs and subs
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;

    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            scan_msg: lidar scan message
            angle: the desired angle in which you want the range

        Returns:
            range: range measurement in meters at the given angle
        */
        //double range = 0.0;
        angle = std::floor((angle - scan_msg->angle_min) / scan_msg->angle_increment); //convert to the closest index value
        
        // if (scan_msg->ranges[angle] ==  'inf')
        //     return scan_msg->range_max;
        // else if (scan_msg->ranges[angle] == 'NaN')
        //         return scan_msg->range_min;
        //     else
        return scan_msg->ranges[angle];
    }

    double get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            scan_msg: sensor scan message from the Lidar
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */
        
        //dist of the lidar along horizontal axis plus theta
        double a = get_range(scan_msg, a_angle);
        //dist of lidar along the car's horizontal axis
        double b = get_range(scan_msg, b_angle);
        //calculate the heading of the car alpha using the two distances a and b
        double alpha = atan2(a * cos(theta) - b, a * sin(theta));
        //Current dist D_t from wall
        double D_t = b * cos(alpha);
        //dist D_t+1 from the wall at a look ahead distance of L
        double D_t_plus_1 = D_t + L * sin(alpha);

        return (dist - D_t_plus_1);
    }

    double pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error

        Returns:
            angle: the estimated streeing angle
        */
        double angle = 0.0;
        integral_error +=error;
        derivative_error = (error - prev_error) / 0.1;
        angle = kp*error + ki*integral_error + kd*derivative_error;

        //limit angle to +-30degrees as the car has that as the max limits
        if (angle<(-30*0.0174533))
            angle = -30*0.0174533;
        else if (angle>(30*0.0174533))
            angle = 30*0.0174533;

        prev_error = error;
        return angle;
    }

    double velocity_limiter(double vel, double angle)
    {
        /*
        Based on the current required steering angle, limit the linear velocity

        Args:
            vel: desired linear velocity based on the error
            angle: estimated streeing angle

        Returns:
            min(vel, upper_limit): a velocity that is bounded by the upper limit
        */
        ///TODO: change the angle limits to tunable parameters read from a params file
        vel +=1;
        if (fabs(angle) < (10.0*0.0174533)) {
            RCLCPP_DEBUG(this->get_logger(), "Limited to max_Speed");
            // return std::min(200, max_velocity);
            return max_velocity;
        }else if (fabs(angle) < (20.0*0.0174533)) {
                RCLCPP_DEBUG(this->get_logger(), "Limited to med_speed");
                // return std::min(200, mid_velocity);
                return mid_velocity;
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "Limited to min_speed");
                    // return std::min(200, min_velocity);
                    return min_velocity;
                }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Lidar Communication Established");
        sampling_time = scan_msg->scan_time;
        double error = get_error(scan_msg, distance_setpoint); 
        // Calcualte the desired steering with PID control
        double steering_angle = pid_control(error);
        //Calculate the velocity as a function of the streeing angle, high velocity for small steering angles
        double velocity = max_velocity - (fabs(steering_angle)/30 * (max_velocity-min_velocity));
        velocity = velocity_limiter(velocity, steering_angle); //set hard upper limits
        
        //Publish the desired drive message to the car over the drive_topic
        RCLCPP_DEBUG(this->get_logger(), "Linear velocity: %f Steering Angle: %f", velocity, steering_angle);
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg = drive_message(steering_angle, acceleration, velocity);
        drive_publisher->publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}