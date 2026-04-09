#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

// 状态变量
bool backing_up = false;
int back_counter = 0;

// 回调函数 - 直接在检测到障碍时触发后退
void rangeCallback1(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Left: %f", msg->range);
    if (msg->range < 0.4 && !backing_up) {
        ROS_WARN("BACK! Left sensor triggered");
        backing_up = true;
        back_counter = 20;  // 1秒 @20Hz
    }
}

void rangeCallback2(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Front: %f", msg->range);
    if (msg->range < 0.4 && !backing_up) {
        ROS_WARN("BACK! Front sensor triggered");
        backing_up = true;
        back_counter = 20;
    }
}

void rangeCallback3(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Right: %f", msg->range);
    if (msg->range < 0.4 && !backing_up) {
        ROS_WARN("BACK! Right sensor triggered");
        backing_up = true;
        back_counter = 20;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tof_avoidance");
    ros::NodeHandle nh;
    
    // 订阅传感器
    ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::Range>("/ul/sensor1", 10, rangeCallback1);
    ros::Subscriber sub_2 = nh.subscribe<sensor_msgs::Range>("/ul/sensor2", 10, rangeCallback2);
    ros::Subscriber sub_3 = nh.subscribe<sensor_msgs::Range>("/ul/sensor3", 10, rangeCallback3);
    
    // 发布速度
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(20);
    
    ROS_INFO("TOF avoidance started. Moving forward...");
    
    while (ros::ok())
    {
        ros::spinOnce();
        
        // 控制逻辑
        if (backing_up) {
            if (back_counter > 0) {
                cmd_vel.linear.x = -0.2;  // 后退
                back_counter--;
            } else {
                backing_up = false;  // 后退结束
                cmd_vel.linear.x = 0.2;  // 恢复直行
                ROS_INFO("Backup complete. Moving forward...");
            }
        } else {
            cmd_vel.linear.x = 0.2;  // 直行
        }
        
        cmd_vel.angular.z = 0;
        cmd_pub.publish(cmd_vel);
        
        rate.sleep();
    }
    
    return 0;
}
