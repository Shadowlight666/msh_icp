#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);

void pclCallbackA(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::fromROSMsg(*cloud_msg, *cloud_a);
}

void pclCallbackB(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::fromROSMsg(*cloud_msg, *cloud_b);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_example");
    ros::NodeHandle nh;

    ros::Subscriber sub_a = nh.subscribe("/pcl_a", 1, pclCallbackA);
    ros::Subscriber sub_b = nh.subscribe("/pcl_b", 1, pclCallbackB);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        if (!cloud_a->empty() && !cloud_b->empty())
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(cloud_a);
            icp.setInputTarget(cloud_b);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);

            if (icp.hasConverged())
            {
                Eigen::Matrix4f transformation = icp.getFinalTransformation();
                std::cout << "Transformation Matrix:" << std::endl;
                for (int i = 0; i < transformation.rows(); ++i)
                {
                    for (int j = 0; j < transformation.cols(); ++j)
                    {
                        std::cout << transformation(i, j) << " ";
                    }
                    std::cout << std::endl;
                }

                // 从 transformation 中提取位姿信息
            }
        }

        loop_rate.sleep();
    }
    return 0;
}
