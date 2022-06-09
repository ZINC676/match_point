//
// Created by zinc on 22-5-25.
//
#include "match.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

Match::Match(){

}

//重载虚函数，启动时自动调用
void Match::onInit()
{
    // test if the nodelet could work
    NODELET_DEBUG("Init nodelet...");
    ROS_INFO("Nodelet is OK for test");

    auto& nh_ = getPrivateNodeHandle();

    getParameters(); // get param from launch file
    for (int i = 0; i < data_num_; ++i) {
        loadAndSavePointcloud(i);
    }
    ROS_INFO("Finish all!");

    getParam(nh_); // get pcd file path and leftSize data from launch file

    // read in file from scanning
    pcl::PointCloud<pointT>::Ptr cloud_from_race (new pcl::PointCloud<pointT>);
    pcl::PointCloud<pointT>::Ptr cloud_from_obj (new pcl::PointCloud<pointT>);
    readInFile(pcd_from_race_, cloud_from_race); // usually put scanning pcd file from race in here
    readInFile(pcd_from_obj_, cloud_from_obj); // usually put obj transform to pcd file in here (already done)

    // filter those useless points
    cloudProcess(cloud_from_race);
    cloudProcess(cloud_from_obj);

    // create publisher object
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 1);

    ros::NodeHandle nh_reconfig(nh_, "match_reconfig");
    srv_ = new dynamic_reconfigure::Server<rm_match::matchConfig>(nh_reconfig);
    dynamic_reconfigure::Server<rm_match::matchConfig>::CallbackType cb = boost::bind(&Match::reconfigCB, this, _1, _2);
    srv_->setCallback(cb);

    // Start processing and publishing point cloud msg
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        pcl::PointCloud<pointT>::Ptr origin_cloud_processed_loop (new pcl::PointCloud<pointT>);
        pcl::PointCloud<pointT>::Ptr target_cloud_processed_loop (new pcl::PointCloud<pointT>);

        pcl::copyPointCloud(*cloud_from_race, *origin_cloud_processed_loop);
        pcl::copyPointCloud(*cloud_from_obj, *target_cloud_processed_loop);

        cout << "Start processing point cloud" << endl;

        // select a 3D square you want to align from source cloud
        squareSelection(origin_cloud_processed_loop);

        // proceed align
        pcl::PointCloud<pointT>::Ptr aligned_cloud (new pcl::PointCloud<pointT>);
        icpAlign(origin_cloud_processed_loop, aligned_cloud, target_cloud_processed_loop);

        // give every points color(rgb)
        pcl::PointCloud<pointT>::Ptr color_origin_cloud (new pcl::PointCloud<pointT>);
        pcl::PointCloud<pointT>::Ptr color_target_cloud (new pcl::PointCloud<pointT>);
        pcl::PointCloud<pointT>::Ptr color_aligned_cloud (new pcl::PointCloud<pointT>);
        changePointColor(origin_cloud_processed_loop, 255, 255, 255, color_origin_cloud);
        changePointColor(target_cloud_processed_loop, 255, 0, 0, color_target_cloud);
        changePointColor(aligned_cloud, 0, 255, 0, color_aligned_cloud);

        // sum the aligned result and two colorful cloud
        pcl::PointCloud<pointT>::Ptr origin_target_aligned_sum (new pcl::PointCloud<pointT>);
        *origin_target_aligned_sum = *color_target_cloud + *color_aligned_cloud + *color_origin_cloud;

        // change msg type to publish in ros
        cout << "Start publishing point colorful data" << endl;

        pcl::toROSMsg(*origin_target_aligned_sum, output_);
        output_.header.frame_id = "cloud";
        pcl_pub_.publish(output_);

        ros::spinOnce();
        loop_rate.sleep();
        printf("\x1b[H\x1b[2J");
    }

    cout << " Publish successfully! " << endl;
    ros::spin();
}

void Match::getParam(const ros::NodeHandle& nh){
    // get pcd file path and leftSize which used to filter point cloud
    nh.param<std::string>("pcd_from_race_", pcd_from_race_, "Null");
    nh.param<std::string>("pcd_from_obj_", pcd_from_obj_, "Null");
    nh.param<double>("leftSize_", leftSize_, 0.01);
//    cout << "File path is: \n" << "race file :" << pcd_from_race_ << '\n' << "obj file is: " <<
//              pcd_from_obj_ << endl;
//    cout << "The leftSize from launch file is: " << leftSize_
//              << endl;
}

void Match::readInFile(const std::string& file_path_read, const pcl::PointCloud<pointT>::Ptr& cloud_from_file) {
//    cout << " start reading pcd file" << endl;
    reader_.read(file_path_read, *cloud_from_file);
//    cout << "pcd file read in successfully! It has :" <<
//     cloud_from_file->points.size() << " points." << endl;
}

// filter point cloud data using voxel grid filter
void Match::cloudProcess(const pcl::PointCloud<pointT>::Ptr& cloud_in){
    // Voxel grid: clean most of the point
    sor_.setInputCloud(cloud_in);
    sor_.setLeafSize(leftSize_, leftSize_, leftSize_);
    sor_.filter(*cloud_in);
//    cout << "PointCloud after filter has: " << cloud_in->points.size()
//     << "points" << endl;
}

// keep those points we want, delete others
void Match::squareSelection(const pcl::PointCloud<pointT>::Ptr& cloud_in){

    pass_.setInputCloud(cloud_in);
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(pass_z_min_, pass_z_max_);
    pass_.setFilterLimitsNegative(false);
    pass_.filter(*cloud_in);

    pass_.setInputCloud(cloud_in);
    pass_.setFilterFieldName("x");
    pass_.setFilterLimits(pass_x_min_, pass_x_max_);
    pass_.setFilterLimitsNegative(false);
    pass_.filter(*cloud_in);

    pass_.setInputCloud(cloud_in);
    pass_.setFilterFieldName("y");
    pass_.setFilterLimits(pass_y_min_, pass_y_max_);
    pass_.setFilterLimitsNegative(false);
    pass_.filter(*cloud_in);
}

// Abandoned function?
void Match::writeFile(const pcl::PointCloud<pointT>::Ptr& cloud_to_write, const std::string& file_name){

    cout << "Cloud to write has: " <<
             cloud_to_write->points.size() << " points. " << endl;

    writer_.write("/home/zinc/ros_ws/src/rm_match/src/align_data/"+ file_name + ".pcd", *cloud_to_write, false);
    cout << "saved in: /home/zinc/ros_ws/src/rm_match/src/align_data/"+ file_name + ".pcd"
              << endl;
}

// Use to print align matrix
void Match::print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// Get param from dynamic_reconfigure
void Match::reconfigCB(rm_match::matchConfig& config, uint32_t level){
    (void)level; // prevent some unnecessary errors from not implemented val

    rot_x_ <<  1, 0, 0, 0,
    0, cos(config.rot_x * PI / 180), -sin(config.rot_x * PI / 180), 0,
    0, sin(config.rot_x * PI / 180), cos(config.rot_x * PI / 180), 0,
    0, 0, 0, 1;
    rot_y_ << cos(config.rot_y * PI / 180), 0, sin(config.rot_y * PI / 180), 0,
    0, 1, 0, 0,
    -sin(config.rot_y * PI / 180), 0, cos(config.rot_y * PI / 180), 0,
    0, 0, 0, 1;
    rot_z_ << cos(config.rot_z * PI / 180), -sin(config.rot_z * PI / 180), 0, 0,
              sin(config.rot_z * PI / 180), cos(config.rot_z * PI / 180), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    trans_ << 1, 0, 0, config.trans_x,
              0, 1, 0, config.trans_y,
              0, 0, 1, config.trans_z,
              0, 0, 0, 1;

    manual_ = trans_ * rot_x_ * rot_y_ * rot_z_;

    pass_z_min_ = config.pass_z_min;
    pass_z_max_ = config.pass_z_max;
    pass_x_min_ = config.pass_x_min;
    pass_x_max_ = config.pass_x_max;
    pass_y_min_ = config.pass_y_min;
    pass_y_max_ = config.pass_y_max;
}

// Using icp to align point cloud
void Match::icpAlign(const pcl::PointCloud<pointT>::Ptr& cloud_to_align,
              const pcl::PointCloud<pointT>::Ptr& cloud_final,
              const pcl::PointCloud<pointT>::Ptr& cloud_target){

    pcl::search::KdTree<pointT>::Ptr treeSrc(new pcl::search::KdTree<pointT>);
    pcl::search::KdTree<pointT>::Ptr treeTarget(new pcl::search::KdTree<pointT>);

    static pcl::IterativeClosestPoint<pointT, pointT> icp;

    treeSrc->setInputCloud(cloud_to_align);
    treeTarget->setInputCloud(cloud_target);
    icp.setInputSource(cloud_to_align);
    icp.setInputTarget(cloud_target);

//    icp.setMaxCorrespondenceDistance(3); //Be careful of this value. It has a large influence to the result of ICP.
    icp.setMaximumIterations (80);
//    icp.setTransformationEpsilon(1e-10);
//    icp.setEuclideanFitnessEpsilon(0.01);
    icp.align(*cloud_final);

    Eigen::Matrix4d aligned_matrix = Eigen::Matrix4d::Identity();
    aligned_matrix = icp.getFinalTransformation ().cast<double>();

    cout << "Align finished, aligned_matrix is here: " << endl;
    print4x4Matrix(aligned_matrix);

    if(aligned_matrix != manual_ * aligned_matrix){
        manualTransform(cloud_final);
        cout << "Manual fixed matrix is here: " << endl;
        print4x4Matrix(manual_ * aligned_matrix);
    }
}

void Match::changePointColor(const pcl::PointCloud<pointT>::Ptr& cloud_in,
                             int color_R, int color_G, int color_B,
                             const pcl::PointCloud<pointT>::Ptr& color_cloud_out){
    for (int i = 0; i <  cloud_in->points.size(); i++)
    {
        pcl::PointXYZRGB  p;
        p.x=cloud_in->points[i].x;
        p.y=cloud_in->points[i].y;
        p.z=cloud_in->points[i].z;

        p.r =  color_R;
        p.g =  color_G;
        p.b =  color_B;
        color_cloud_out->points.push_back(p);
    }
}

// Add manual change to align_matrix and aligned cloud
void Match::manualTransform(const pcl::PointCloud<pointT>::Ptr& cloud_in){
    pcl::transformPointCloud(*cloud_in, *cloud_in, manual_);
}



// Codes down there is for transforming rosbog file to pcd file.
void Match::loadAndSavePointcloud(int index) {
    string path = input_bag_path_ + int2str(index) + ".bag";
    fstream file_;
    file_.open(path, ios::in);
    if (!file_) {
        cout << "File " << path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;
    for (const rosbag::MessageInstance& m : view) {
        livox_cloud_ = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type

        for(uint i = 0; i < livox_cloud_.point_num; ++i) {
            pointData myPoint;
            myPoint.x = livox_cloud_.points[i].x;
            myPoint.y = livox_cloud_.points[i].y;
            myPoint.z = livox_cloud_.points[i].z;
            myPoint.i = livox_cloud_.points[i].reflectivity;

            vector_data_.push_back(myPoint);
        }
        ++cloudCount;
        if (cloudCount >= threshold_lidar_) {
            break;
        }
    }
    dataSave(index);
    vector_data_.clear();
}

void Match::writeTitle(const string filename, unsigned long point_num) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    ROS_INFO("Save file %s", filename.c_str());
}

void Match::writePointCloud(const string filename, const vector<pointData> singlePCD) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        for (unsigned long i = 0; i < singlePCD.size(); ++i) {
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
        }
    }
}

void Match::dataSave(int index) {
    string outputName = output_path_ + int2str(index) + ".pcd";
    writeTitle(outputName, vector_data_.size());
    writePointCloud(outputName, vector_data_);
}

void Match::getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path_)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    else {
        cout << input_bag_path_ << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path_)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar_)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("data_num", data_num_)) {
        cout << "Can not get the value of data_num" << endl;
        exit(1);
    }
}

PLUGINLIB_EXPORT_CLASS(Match, nodelet::Nodelet); // first argument is class name, the second arg is father class name



//pcl::copyPointCloud(*cloud, indexs, *cloudOut);//中间的indexs是需要拷贝的点云的索引，*cloud为原始点云，*cloudOut为新点云