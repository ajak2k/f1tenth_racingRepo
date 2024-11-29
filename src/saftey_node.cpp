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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "utils.hpp"

#include <cmath>
#include <limits>

//Define namespaces
using namespace std;
using nav_msgs::msg::Odometry;
using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using namespace std::chrono_literals; //for the ability to define time as xxxms and s directly


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: ivestigate QoS for sub and pub to get more reliable code
        RCLCPP_INFO(this->get_logger(), "Constructing saftey_node"); 

        //Subscriptions to the car to get odomerty and Lidar Data
        odom_subscription = this->create_subscription<Odometry>("/ego_racecar/odom", 10, bind(&Safety::odom_callback, this, _1));
        scan_subscription = this->create_subscription<LaserScan>("/scan", 10, bind(&Safety::scan_callback, this, _1));

        //Publisher to the car drive topic
        drive_publisher = this->create_publisher<AckermannDriveStamped>("drive", 10);
    }

private:
    //Declaration of Member Variables
    /// TODO: Update this to move the constant values to a config file
    float iTTC_Threshold = 0.35;
    float brake_accel = -10.0;
    Odometry CarPose;

    //pubs and subs and timers
    rclcpp::Subscription<Odometry>::SharedPtr odom_subscription;
    rclcpp::Subscription<LaserScan>::SharedPtr  scan_subscription;
    rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_publisher;

    void odom_callback(const Odometry::ConstSharedPtr msg)
    {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Odometry Communication Established");
        this->CarPose = *msg;
    }

    void scan_callback(const LaserScan::ConstSharedPtr scan_msg) 
    {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Lidar Scan Communication Established");
        //If the current scan indicates not safe operation,stop the car with a 0 speed, 0 steering angle cmd
        if (!safety_check(scan_msg))
        {   
            RCLCPP_WARN(this->get_logger(), "Unsafe Operation, Applying Brakes");
            drive_publisher->publish(drive_message(0.0,brake_accel,0.0));
        }
    }

    bool safety_check(const LaserScan::ConstSharedPtr lidar_scan)
    {
        //set initial ttc to infinity(or here the max of a float)
        float iTTC = std::numeric_limits<float>::infinity();
        /// TODO: What is we only 'look forward'(restric the lidar range to less than min to max) to improve performance in narrow tracks?
        float ScanAngle = lidar_scan->angle_min;
        float relativeSpeed = 0.0;

        for (long unsigned int i=0; i<lidar_scan->ranges.size(); i++)
        {
            if (lidar_scan->ranges[i]>0.0) //reject junk range values
            {
                ScanAngle += lidar_scan->angle_increment;
                relativeSpeed = this->CarPose.twist.twist.linear.x * std::cos(ScanAngle);
                if (relativeSpeed > 0)
                 iTTC = std::min(lidar_scan->ranges[i]/relativeSpeed, iTTC);
            }
        }
        if (iTTC < this->iTTC_Threshold)
            return false;
        else 
            return true;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}