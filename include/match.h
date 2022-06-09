//
// Created by zinc on 22-5-25.
//

#ifndef RM_MATCH_MATCH_H
#define RM_MATCH_MATCH_H


#include <nodelet/nodelet.h>

#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ctime>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>

#include <rm_match/matchConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "CustomMsg.h"
#include "common.h"

#define PI acos(-1.0)

typedef pcl::PointXYZRGB pointT;
//using namespace std;
struct pointData{
    float x;
    float y;
    float z;
    int i;
};

class Match :public nodelet::Nodelet  //继承父类nodelet::Nodelet
{
public:
    Match();         //构造函数，可有可无？
public:
    void onInit() override;   //这个虚函数，在启动本Nodelet节点时，自动调用

    void getParam(const ros::NodeHandle& nh);

    void readInFile(const std::string& file_path_read, const pcl::PointCloud<pointT>::Ptr& cloud_from_file);

    void writeFile(const pcl::PointCloud<pointT>::Ptr& origin_cloud, const std::string& file_name);

    static void print4x4Matrix (const Eigen::Matrix4d & matrix);

    void icpAlign(const pcl::PointCloud<pointT>::Ptr& cloud_in,
                  const pcl::PointCloud<pointT>::Ptr& cloud_final,
                  const pcl::PointCloud<pointT>::Ptr& cloud_target
                  );

    static void changePointColor(const pcl::PointCloud<pointT>::Ptr& cloud_in, int color_R, int color_G, int color_B,
                          const pcl::PointCloud<pointT>::Ptr& color_cloud_out);

    void cloudProcess(const pcl::PointCloud<pointT>::Ptr& cloud_in);

    void squareSelection(const pcl::PointCloud<pointT>::Ptr& cloud_in);

    void reconfigCB(rm_match::matchConfig& config, uint32_t level);

    void manualTransform(const pcl::PointCloud<pointT>::Ptr& cloud_in);


    void loadAndSavePointcloud(int index);
    void writeTitle(const string filename, unsigned long point_num);
    void writePointCloud(const string filename, const vector<pointData> singlePCD);
    void dataSave(int index);
    void getParameters();
private:
    ros::NodeHandle nh_;
    pcl::PCDReader reader_;
    pcl::PassThrough<pointT> pass_;
    pcl::PCDWriter writer_;
    ros::Subscriber pcl_sub_;
    ros::Publisher pcl_pub_;
    pcl::IterativeClosestPoint<pointT, pointT> icp_;
    sensor_msgs::PointCloud2 output_;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor_;

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg_;

    dynamic_reconfigure::Server<rm_match::matchConfig>* srv_{};

    vector<pointData> vector_data_;
    livox_ros_driver::CustomMsg livox_cloud_;
    string input_bag_path_, output_path_;
    int threshold_lidar_, data_num_;

private:
    float depth_limit_{};

    Eigen::Matrix4d rot_x_;
    Eigen::Matrix4d rot_y_;
    Eigen::Matrix4d rot_z_;
    Eigen::Matrix4d trans_;
    Eigen::Matrix4d manual_;

    std::string pcd_from_race_;
    std::string pcd_from_obj_;

    double rot_x_angle_{};
    double rot_y_angle_{};
    double rot_z_angle_{};
    double tran_in_x{};
    double tran_in_y{};
    double tran_in_z{};

    double pass_z_min_{};
    double pass_z_max_{};
    double pass_x_min_{};
    double pass_x_max_{};
    double pass_y_min_{};
    double pass_y_max_{};

    double leftSize_{};
};





#endif //RM_MATCH_MATCH_H
