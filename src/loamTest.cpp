// Author:   Tong Qin               qintonguav@gmail.com
//           Shaozu Cao             saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "loamFeature.h"
#include "loamScanMatching.h"
#include "laggedMatcher.h"


std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

void process_gt(const std::string &line,
                Eigen::Quaterniond &q,
                Eigen::Vector3d &t)
{
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::stringstream pose_stream(line);
    std::string s;
    Eigen::Matrix<double, 3, 4> gt_pose;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            std::getline(pose_stream, s, ' ');
            gt_pose(i, j) = stof(s);
        }
    }
    Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
    q = q_transform * q_w_i;
    q.normalize();
    t = q_transform * gt_pose.topRightCorner<3, 1>();
}


struct PointCloudPublisher
{
    PointCloudPublisher(const std::string &name, 
                        const std::string &frame,
                        ros::NodeHandle &n)
        : frame_(frame)
    {
        pub_laser_cloud_ = n.advertise<sensor_msgs::PointCloud2>(name, 2);
    }

    void publish_pc(const pcl::PointCloud<PointType> &pc)
    {
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(pc, pc_msg);
        // pc_msg.header.stamp = ros::Time().fromSec(timestamp);
        pc_msg.header.frame_id = frame_;
        pub_laser_cloud_.publish(pc_msg);
    }

    std::string frame_;
    ros::Publisher pub_laser_cloud_;
};

struct OdometryPublisher
{
    OdometryPublisher(const std::string odo_cname,
                      const std::string path_cname,
                      const std::string header_frame,
                      const std::string child_frame,
                      ros::NodeHandle &n)
        : header_frame_(header_frame), child_frame_(child_frame)
    {
        pubLaserOdometry_ = n.advertise<nav_msgs::Odometry>(odo_cname, 100);
        pubLaserPath_ = n.advertise<nav_msgs::Path>(path_cname, 100);
    }

    void publish(const Eigen::Quaterniond &q_w_curr,
                 const Eigen::Vector3d &t_w_curr)
    {
        // publish odometry
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = header_frame_;
        laserOdometry.child_frame_id = child_frame_;
        // laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        pubLaserOdometry_.publish(laserOdometry);

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        laserPath_.header.stamp = laserOdometry.header.stamp;
        laserPath_.poses.push_back(laserPose);
        laserPath_.header.frame_id = header_frame_;
        pubLaserPath_.publish(laserPath_);
    }


    ros::Publisher pubLaserOdometry_;
    ros::Publisher pubLaserPath_;
    nav_msgs::Path laserPath_;

    std::string header_frame_;
    std::string child_frame_;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    // ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);


    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);
    std::cout << "reading timestamp file from: " << dataset_folder + timestamp_path << std::endl;

    std::string ground_truth_path = "results/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);
    std::cout << "reading gt file from: " << dataset_folder + ground_truth_path << std::endl;

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);

    PointCloudPublisher scan_pub("/velodyne_points", "/camera_init", n);
    PointCloudPublisher flat_pub("/flat", "/camera_init", n);
    PointCloudPublisher less_flat_pub("/less_flat", "/camera_init", n);
    PointCloudPublisher curv_pub("/curv", "/camera_init", n);
    PointCloudPublisher less_curv_pub("/less_curv", "/camera_init", n);
    OdometryPublisher odo_pub("/laser_odom_to_init", "/laser_odom_path", "/camera_init", "/laser_odom", n);
    OdometryPublisher gt_pub("/odometry_gt", "/path_gt", "/camera_init", "/ground_truth", n);


    // LOAMScanMatching matcher;
    lagged::LaggedScanMatching matcher;

    // // TODO: size of pose = size of lidar bin + 1??
    // std::getline(ground_truth_file, line);

    while (std::getline(timestamp_file, line) && ros::ok())
    {
        // float timestamp = stof(line);

        // gt
        std::getline(ground_truth_file, line);
        Eigen::Quaterniond q_gt;
        Eigen::Vector3d t_gt;
        process_gt(line, q_gt, t_gt);
        gt_pub.publish(q_gt, t_gt);

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::cout << "Reading lidar from: " << lidar_data_path.str() << std::endl;
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        scan_pub.publish_pc(laser_cloud);

        line_num ++;

        std::vector<pcl::PointCloud<PointType>> lines = KittiDataPreprocessing().convertScanToVectorOfLines(laser_cloud);
        
        std::cout << "extractFeatures..." << std::endl;
        // LidarFeatures features = LidarFeatureExtraction().extractFeatures(lines);
        LidarFeatures features = LidarFeatureExtraction().extractFeaturesSimple(lines);


        flat_pub.publish_pc(*features.surfPointsFlat);
        less_flat_pub.publish_pc(*features.surfPointsLessFlat);
        curv_pub.publish_pc(*features.cornerPointsSharp);
        less_curv_pub.publish_pc(*features.cornerPointsLessSharp);

        matcher.match(features);

        Eigen::Quaterniond q;
        Eigen::Vector3d t;

        matcher.getPose(q, t);
        odo_pub.publish(q, t);

        std::cout << "t b2w :" << t << std::endl;


        r.sleep();
    }
    // bag_out.close();
    std::cout << "Done \n";


    return 0;
} 