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
double start_yaw = 0;
double last_yaw = 0;
double turn = 0;  // 累积转角
int stop_counter = 0;

void odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    
    if (is_turning)
    {
        // 计算当前yaw与上一次yaw的差值，考虑跳变
        double delta = yaw - last_yaw;
        if (delta > M_PI) delta -= 2*M_PI;
        if (delta < -M_PI) delta += 2*M_PI;
        
        turn += delta;  // 累积转角
        last_yaw = yaw;
        
        ROS_INFO("Yaw: %.2f, Turn: %.2f", yaw, turn);
        
        // 累积转角达到180度（3.14弧度）
        if (turn >= 3.04) {
            stop_counter = 20;
            is_turning = false;
            turn = 0;  // 清零
            ROS_INFO("-> STOP");
        }
    }
    else
    {
        // 不在旋转时，记录当前yaw作为下一次旋转的起点
        last_yaw = yaw;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_180");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/odom", 10, odom_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(20);
    
    // 等待一下获取初始角度
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    
    ROS_INFO("Starting rotation...");
    
    while (ros::ok())
    {
        if (stop_counter > 0)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            stop_counter--;
            
            if (stop_counter == 0) {
                // 停止结束，开始下一次旋转
                is_turning = true;
                turn = 0;  // 重置累积转角
                ROS_INFO("-> TURN");
            }
        }
        else if (!is_turning)
        {
            // 初始状态，开始第一次旋转
            is_turning = true;
            turn = 0;  // 重置累积转角
            ROS_INFO("-> TURN");
        }
        
        if (is_turning)
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
