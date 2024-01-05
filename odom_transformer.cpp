#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix4f transformation_matrix; // 假设这是你获得的变换矩阵
ros::Publisher pub_transformed_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // 从 /odom_a 获取姿态信息
    Eigen::Vector3f position(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);

    Eigen::Quaternionf quaternion(odom_msg->pose.pose.orientation.w,
                                  odom_msg->pose.pose.orientation.x,
                                  odom_msg->pose.pose.orientation.y,
                                  odom_msg->pose.pose.orientation.z);

    // 构建变换
    Eigen::Affine3f odom_affine = Eigen::Translation3f(position) * Eigen::Quaternionf(quaternion);
    Eigen::Matrix4f odom_matrix = odom_affine.matrix();

    // 使用变换矩阵进行转换
    Eigen::Matrix4f transformed_matrix = odom_matrix * transformation_matrix;

    // 从转换后的矩阵中提取姿态信息
    Eigen::Vector3f transformed_position = transformed_matrix.block<3, 1>(0, 3);
    Eigen::Quaternionf transformed_quaternion(transformed_matrix.block<3, 3>(0, 0));

    // 发布新的转换后的odom信息
    nav_msgs::Odometry transformed_odom_msg;
    transformed_odom_msg.header = odom_msg->header;
    transformed_odom_msg.pose.pose.position.x = transformed_position.x();
    transformed_odom_msg.pose.pose.position.y = transformed_position.y();
    transformed_odom_msg.pose.pose.position.z = transformed_position.z();
    transformed_odom_msg.pose.pose.orientation.w = transformed_quaternion.w();
    transformed_odom_msg.pose.pose.orientation.x = transformed_quaternion.x();
    transformed_odom_msg.pose.pose.orientation.y = transformed_quaternion.y();
    transformed_odom_msg.pose.pose.orientation.z = transformed_quaternion.z();

    pub_transformed_odom.publish(transformed_odom_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_transformer");
    ros::NodeHandle nh;

    // 订阅 /odom_a 话题
    ros::Subscriber sub_odom = nh.subscribe("/odom_a", 1, odomCallback);

    // 发布到 /odom_a2b 话题
    pub_transformed_odom = nh.advertise<nav_msgs::Odometry>("/odom_a2b", 1);

    ros::spin();

    return 0;
}
