#pragma once

#include <stdio.h>

#include <boost/bind.hpp>
#include <cmath>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <random>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/sdf_api_wrapper.hpp"

namespace gazebo {

static const std::string kDefaultParentFrameId = "world";
static const std::string kDefaultChildFrameId = "odometry_sensor";
static const std::string kDefaultLinkName = "odometry_sensor_link";
static const std::string kDefaultJointName = "odometry_sensor_joint";
static const std::string kDefaultPosePubTopic = "ball/pose";

class GazeboCablePlugin : public ModelPlugin {
 public:
  GazeboCablePlugin()
      : ModelPlugin(),
        parent_frame_id_(kDefaultParentFrameId),
        child_frame_id_(kDefaultChildFrameId),
        link_name_(kDefaultLinkName),
        joint_name_(kDefaultJointName),
        pose_pub_topic_(kDefaultPosePubTopic),
        pubs_and_subs_created_(false),
        ball_radius_(0.0),
        ball_weight_(0.0),
        cable_length_(0.0) {}

  ~GazeboCablePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  bool pubs_and_subs_created_;
  void CreatePubsAndSubs();

  std::string namespace_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  std::string link_name_;
  std::string joint_name_;

  std::string pose_pub_topic_;

  double ball_radius_;
  double ball_weight_;
  double cable_length_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr pose_pub_;
  gazebo::transport::PublisherPtr broadcast_transform_pub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;

  event::ConnectionPtr updateConnection_;
  boost::thread callback_queue_thread_;
  void QueueThread();
};

};  // namespace gazebo