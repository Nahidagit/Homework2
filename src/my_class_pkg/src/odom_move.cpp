#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

double normalize_angle(double angle) {
    while (angle > M_PI)  angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

bool is_turning = false;
double start_x = 0, start_y = 0, start_yaw = 0;
double dist = 0, turn = 0;
int stop_counter = 0;

void odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    
    if (!is_turning)
    {
        dist = std::sqrt((x-start_x)*(x-start_x) + (y-start_y)*(y-start_y));
        
        if (dist > 1.0) {
            stop_counter = 20;
            is_turning = true;
            start_yaw = yaw;
            ROS_INFO("-> TURN");
        }
    }
    else
    {
        turn = normalize_angle(yaw - start_yaw);
        
        if (turn > 1.52) {
            stop_counter = 10;
            is_turning = false;
            start_x = x;
            start_y = y;
            start_yaw = yaw;
            ROS_INFO("-> GO");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smooth_square");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/odom", 10, odom_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(20);
    
    while (ros::ok())
    {
        if (stop_counter > 0)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            stop_counter--;
        }
        else if (!is_turning)
        {
            // 直行减速：根据剩余距离调整速度
            double remaining = 1.0 - dist;  // 还剩多少米
            
            if (remaining < 0.2)
                cmd_vel.linear.x = 0.1;    // 最后20cm：龟速
            else if (remaining < 0.4)
                cmd_vel.linear.x = 0.2;     // 40-20cm：慢速
            else if (remaining < 0.6)
                cmd_vel.linear.x = 0.3;    // 60-40cm：中速
            else
                cmd_vel.linear.x = 0.3;      // 全速
            
            cmd_vel.angular.z = 0;
        }
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0.3;
        }
        
        pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
