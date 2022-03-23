#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/mapping/front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

    bool Run();

  private:
    bool InitPose();

    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

    bool HasNewKeyFrame();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;

    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;

    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_pose_data_buff_;

    CloudData current_cloud_data_;
    PoseData  init_pose_;

    CloudData::CLOUD_PTR current_scan_ptr_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    bool pose_inited_ = false;
};
}

#endif
