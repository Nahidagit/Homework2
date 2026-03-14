#include <ros/ros.h>
#include <std_msgs/String.h>
int main(int argc, char **argv)
{
// 初始化 ROS 节点
ros::init(argc, argv, "my_publisher");
// 创建节点句柄
ros::NodeHandle nh;
// 定义一个发布者对象
ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);
//定义一个 ros 频率，间歇 1.0 秒
ros::Rate rate(1.0);
while (ros::ok())
{
// 创建一个消息对象并填充数据
std_msgs::String msg;
msg.data = "Hello, world!";
// 发布消息
pub.publish(msg);
//间歇休息 1 秒
rate.sleep();
}
return 0;
}
