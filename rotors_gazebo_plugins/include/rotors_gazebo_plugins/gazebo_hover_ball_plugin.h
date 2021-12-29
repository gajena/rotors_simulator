#pragma once

#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/gazebo_ros_interface_plugin.h"
#include "rotors_gazebo_plugins/sdf_api_wrapper.hpp"

namespace gazebo {

static const std::string kDefaultPosePubTopic = "state";
static const std::string kDefaultLoadSubTopic = "force";
static const std::string kDefaultParentFrameId = "world";
static const std::string kDefaultChildFrameId = "ball_link";

class GazeboHoverBallPlugin : public ModelPlugin {
 public:
  GazeboHoverBallPlugin()
      : ModelPlugin(),
        state_pub_topic_(kDefaultPosePubTopic),
        load_sub_topic_(kDefaultLoadSubTopic),
        parent_frame_id_(kDefaultParentFrameId),
        child_frame_id_(kDefaultChildFrameId),
        pubs_and_subs_created_(false),
        ball_mass_(0.0),
        payload_mass_(0.0),
        force_(0.0) {}

  ~GazeboHoverBallPlugin(){};

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  bool pubs_and_subs_created_;
  void CreatePubsAndSubs();

  std::string namespace_;
  std::string state_pub_topic_;
  std::string load_sub_topic_;
  std::string parent_frame_id_;
  std::string child_frame_id_;

  double ball_mass_;
  double payload_mass_;
  double force_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr state_pub_;

  gazebo::transport::SubscriberPtr load_sub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;

  event::ConnectionPtr updateConnection_;

  ignition::math::Vector3d load_;

  void LoadCallback(GzWindSpeedMsgPtr& load_msg);
};

};  // namespace gazebo