#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/n_frame", "/lidar_frame", 100);
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", "/n_frame", 100);
    front_end_ptr_ = std::make_shared<FrontEnd>();

    current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    if (!InitPose())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::InitPose() { 
    if (!pose_inited_) {
        if(gnss_pose_data_buff_.size() >= 2) {
            init_pose_ = gnss_pose_data_buff_.back();
            pose_inited_ = true;
        }
    }

    return pose_inited_;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    if(!pose_inited_)
        gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {

    if (cloud_data_buff_.size() == 0)
        return false;

    return true;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;

    if (!odometry_inited) {
        odometry_inited = true;
        // init lidar odometry:
        front_end_ptr_->SetInitPose(init_pose_.pose);
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    laser_odometry_ = Eigen::Matrix4f::Identity();
    // update lidar odometry using current undistorted measurement:
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    cloud_pub_ptr_->Publish(current_scan_ptr_);

    return true;
}
}