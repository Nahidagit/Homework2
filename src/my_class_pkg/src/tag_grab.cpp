#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mgrab_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    // 等待所有服务端就绪
    arm_move_open_client.waitForExistence();
    arm_zero_client.waitForExistence();
    arm_grab_client.waitForExistence();
    arm_release_client.waitForExistence();
    ros::Duration(0.5).sleep();

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ROS_INFO("tf coordinate transforming....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1 = buffer.lookupTransform("arm_base_link", "tag_1", ros::Time(0), ros::Duration(100));

    // 单位转换，ros坐标系到逆运算坐标系
    int x = -int(tfs_1.transform.translation.y * 1000);
    int y = int(tfs_1.transform.translation.x * 1000) + 30;
    int z = int(tfs_1.transform.translation.z * 1000 + 40);
    
    ROS_INFO("Target X: %d, Target Y: %d, Target Z: %d", x, y, z);
    std_srvs::Empty empty_srv;
    
    // 第一步，打开夹爪
    if (arm_release_client.call(empty_srv)) {
        ROS_INFO("Release succeeded");
    } else {
        ROS_ERROR("Release failed");
    }
    sleep(5.0);

    // 第二步，运动到抓取位置
    upros_message::ArmPosition move_srv;
    move_srv.request.x = x;
    move_srv.request.y = y;
    move_srv.request.z = z;
    if (arm_move_open_client.call(move_srv)) {
        ROS_INFO("Move to grasp succeeded");
    } else {
        ROS_ERROR("Move to grasp failed");
    }
    sleep(5.0);

    // 第三步，闭合夹爪
    if (arm_grab_client.call(empty_srv)) {
        ROS_INFO("Grab succeeded");
    } else {
        ROS_ERROR("Grab failed");
    }
    sleep(5.0);

    // 第四步，返回零位
    if (arm_zero_client.call(empty_srv)) {
        ROS_INFO("Zero succeeded");
    } else {
        ROS_ERROR("Zero failed");
    }
    sleep(5.0);

    ros::shutdown();
    return 0;
}
