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

#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "utils.hpp"

constexpr double DEG_TO_RAD = M_PI / 180.0; // Conversion factor for degrees to radians

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
    double kp = 1.0;
    double kd = 1.0;
    double ki = 0.0;

    double error = 0.0;
    double prev_error = 0.0;
    double integral_error = 0.0;
    double derivative_error = 0.0;
    
    double t_0 = -1.0;
    double t = 0.0;
    double t_minus_1 = 0.0;

    //Velocity limits
    double max_velocity = 7.5;
    double mid_velocity = 5.0;
    double mid_velocity2 = 2.0;
    double mid_velocity3 = 1.5;
    double min_velocity = 0.5;
    //double acceleration = 0.0;

    //wall follow parameters
    double L = 1.5; //look ahead distance; remember car length is 0.5m
    double distance_setpoint = 1.0; //desired dist from the wall
    //assume angle of the car is = 0 if heading is straight ahead
    //all angles specified in degrees and then converted to radians, degree * DEG_TO_RAD = radians
    double a_angle = DEG_TO_RAD * (-50); //angle of the beam that is theta degrees ahead of the x-axis of the car
    double b_angle = DEG_TO_RAD * -90; //angle of the beam that is along the x-axis of the car. Set to +90 to follow right wall, set to -90 to follow left wall
    double theta   = DEG_TO_RAD * 40; //angle b/w a and b
    

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
        assert(angle >= scan_msg->angle_min && angle <= scan_msg->angle_max); // Angle must be within range
        int i = std::floor((angle - scan_msg->angle_min) / scan_msg->angle_increment); //convert to the closest index value
        
        //Handle NaN and inf values for the range and retun the max range
        if (std::isnan(scan_msg->ranges[i]) || scan_msg->ranges[i] > scan_msg->range_max)
            return scan_msg->range_max;
        else
            return scan_msg->ranges[i];
    }

    void get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist_sp)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            scan_msg: sensor scan message from the Lidar
            dist: desired distance to the wall

        Returns:
            NA
        */
        
        //a = dist of the lidar along horizontal axis plus theta
        //b = dist of lidar along the car's horizontal axis
        //alpha = the heading of the car alpha using the two distances a and b
        //D_t = Current dist from wall
        //D_t+1 = distance from the wall at a look ahead distance of L
        double a = get_range(scan_msg, a_angle);
        double b = get_range(scan_msg, b_angle);
        double alpha = atan2(a * cos(theta) - b, a * sin(theta));
        double D_t = b * cos(alpha);
        double D_t_plus_1 = D_t + L * sin(alpha);
        //update global error variables
        this -> prev_error = this->error;
        this -> error = dist_sp - D_t_plus_1;
        this -> integral_error += this->error;
        //update time parameters
        this->t_minus_1 = this->t;
        this->t = (double)scan_msg->header.stamp.sec + 
                  (double)scan_msg->header.stamp.nanosec * 1e-9;
        if (this->t_0 == -1.0) this->t_0 = this->t; //if it is the first run, set start time t_0 as the current time t
    }

    double pid_control()
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            N/A, uses all global variables

        Returns:
            angle: the estimated streeing angle
        */
        double delta_time = this->t - this->t_minus_1;
        //Calculate the steering angle
        double angle = this->kp * this->error + 
                       this->ki * this->integral_error * delta_time + 
                       this->kd * this->derivative_error / delta_time;
        //limit angle to +-30degrees as the car has that as the max limits
        angle = std::clamp(angle, -30 * DEG_TO_RAD, 30 * DEG_TO_RAD);
        return angle;
    }

    double velocity_limiter(double angle)
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

        if ((fabs(angle) > 0) && (fabs(angle) < (10.0*DEG_TO_RAD))) {
            RCLCPP_DEBUG(this->get_logger(), "Set to max_Speed");
            return max_velocity;
        }else if ((fabs(angle) > (10.0*DEG_TO_RAD)) && fabs(angle) < (15.0*DEG_TO_RAD)) {
                  RCLCPP_DEBUG(this->get_logger(), "Limited to med_speed");
                  return mid_velocity;
                } else if ((fabs(angle) > (15.0*DEG_TO_RAD)) && fabs(angle) < (20.0*DEG_TO_RAD)) {
                  RCLCPP_DEBUG(this->get_logger(), "Limited to med_speed2");
                  return mid_velocity2;
                } if ((fabs(angle) > (20.0*DEG_TO_RAD)) && fabs(angle) < (25.0*DEG_TO_RAD)) {
                  RCLCPP_DEBUG(this->get_logger(), "Limited to med_speed3");
                  return mid_velocity;
                }else {
                        RCLCPP_DEBUG(this->get_logger(), "Limited to min_speed");
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
        get_error(scan_msg, distance_setpoint); 
        // Calcualte the desired steering with PID control
        double steering_angle = pid_control();
        //Calculate the velocity as a function of the streeing angle, high velocity for small steering angles
        double velocity = velocity_limiter(steering_angle);
        
        //Publish the desired drive message to the car over the drive_topic
        RCLCPP_DEBUG(this->get_logger(), "Linear velocity: %f Steering Angle: %f \n", velocity, steering_angle);
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        ///TODO: this drive_message() is not working, need to investigate
        //drive_msg = drive_message(steering_angle, acceleration, velocity);
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steering_angle;
        this->drive_publisher->publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}