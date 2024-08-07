//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo/plugins/RayPlugin.hh>
#include "livox_ode_multiray_shape.h"
#include <sensor_msgs/msg/point_cloud.hpp>

namespace gazebo {
struct AviaRotateInfo {
    double time;
    double azimuth;
    double zenith;
};

class LivoxPointsPlugin : public RayPlugin {
 public:
    LivoxPointsPlugin();

    virtual ~LivoxPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

   //  GZ_REGISTER_MODEL_PLUGIN(LivoxPointsPlugin)

 private:
    ignition::math::Angle AngleMin() const;

    ignition::math::Angle AngleMax() const;

    double GetAngleResolution() const;

    double AngleResolution() const;

    double GetRangeMin() const;

    double RangeMin() const;

    double GetRangeMax() const;

    double RangeMax() const;

    double GetRangeResolution() const;

    double RangeResolution() const;

    int GetRayCount() const;

    int RayCount() const;

    int GetRangeCount() const;

    int RangeCount() const;

    int GetVerticalRayCount() const;

    int VerticalRayCount() const;

    int GetVerticalRangeCount() const;

    int VerticalRangeCount() const;

    ignition::math::Angle VerticalAngleMin() const;

    ignition::math::Angle VerticalAngleMax() const;

    double GetVerticalAngleResolution() const;

    double VerticalAngleResolution() const;

 protected:
    virtual void OnNewLaserScans();

 private:
    void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                        boost::shared_ptr<physics::LivoxOdeMultiRayShape>& ray_shape);

    void InitializeScan(msgs::LaserScan*& scan);

    void SendRosTf(const ignition::math::Pose3d& pose, const std::string& father_frame, const std::string& child_frame);

    boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
    gazebo::physics::CollisionPtr laserCollision;
    physics::EntityPtr parentEntity;
    transport::PublisherPtr scanPub;
    sdf::ElementPtr sdfPtr;
    msgs::LaserScanStamped laserMsg;
    transport::NodePtr node;
    gazebo::sensors::SensorPtr raySensor;
    std::vector<AviaRotateInfo> aviaInfos;

    std::shared_ptr<rclcpp::Node> rosNode;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr rosPointPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    int64_t samplesStep = 0;
    int64_t currStartIndex = 0;
    int64_t maxPointSize = 1000;
    int64_t downSample = 1;

    double maxDist = 400.0;
    double minDist = 0.1;
};

}  // namespace gazebo

#endif  // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
