#pragma once

#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/sdf_api_wrapper.hpp"

namespace gazebo {

static const std::string kDefaultPosePubTopic = "ball/pose";

class GazeboCablePlugin : public ModelPlugin {
 public:
  GazeboCablePlugin()
      : ModelPlugin(),
        pose_pub_topic_(kDefaultPosePubTopic),
        pubs_and_subs_created_(false),
        ball_mass_(0.0),
        mav_mass_(0.0),
        cable_length_(0.0) {}

  ~GazeboCablePlugin() {};

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  bool pubs_and_subs_created_;
  void CreatePubsAndSubs();

  std::string namespace_;
  std::string pose_pub_topic_;

  double mav_mass_;
  double ball_mass_;
  double cable_length_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr pose_pub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;

  event::ConnectionPtr updateConnection_;
};

};  // namespace gazebo