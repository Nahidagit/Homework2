#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

bool collision = false;
bool backing_up = false;  // 新增后退状态标志
int back_counter = 0;

void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    for (int i = 0; i < msg->data.size(); ++i) {
        if (msg->data[i] != 0) {
            collision = true;  // 只设置碰撞标志，不在回调里做其他事
            break;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bumper_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber bump_sub = nh.subscribe("/robot/bump_sensor", 10, bumpCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(20);
    
    ROS_INFO("Bumper controller started.");
    
    while (ros::ok())
    {
        ros::spinOnce();
        
        // 碰撞检测和状态切换
        if (collision && !backing_up) {
            backing_up = true;
            back_counter = 20;  // 设置后退时间
            ROS_INFO("Collision! Backing up...");
            collision = false;  // 重置碰撞标志
        }
        
        // 控制逻辑
        if (backing_up) {
            if (back_counter > 0) {
                cmd_vel.linear.x = -0.2;  // 后退
                back_counter--;
            } else {
                backing_up = false;  // 后退结束
                cmd_vel.linear.x = 0.2;  // 恢复直行
                ROS_INFO("Backup complete.");
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
