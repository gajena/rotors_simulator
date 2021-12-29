#include "rotors_gazebo_plugins/gazebo_hover_ball_plugin.h"

#include <rotors_gazebo_plugins/common.h>

#include <iostream>

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "Vector3dStamped.pb.h"

namespace gazebo {
void GazeboHoverBallPlugin::Load(physics::ModelPtr _model,
                                 sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_hover_ball_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("linkName")) {
    std::string link_name = _sdf->GetElement("linkName")->Get<std::string>();
    link_ = model_->GetLink(link_name);
    if (link_ == NULL) {
      gzthrow("[gazebo_hover_ball_plugin] Couldn't find specified link \""
              << link_name << "\".");
    }
  } else {
    gzthrow("[gazebo_hover_ball_plugin] Please specify a linkName.\n");
  }

  getSdfParam<double>(_sdf, "ballMass", ball_mass_, ball_mass_);
  payload_mass_ = ball_mass_;

  getSdfParam<std::string>(_sdf, "statePubTopic", state_pub_topic_,
                           state_pub_topic_);
  getSdfParam<std::string>(_sdf, "inputLoadSubTopic", load_sub_topic_,
                           load_sub_topic_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboHoverBallPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboHoverBallPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  ignition::math::Pose3d position = link_->WorldCoGPose();
  ignition::math::Vector3d velocity = link_->WorldLinearVel();
  ignition::math::Vector3d accel = link_->WorldLinearAccel();

  gz_geometry_msgs::Vector3dStamped state_msg;
  state_msg.mutable_header()->set_frame_id("world");
  state_msg.mutable_header()->mutable_stamp()->set_sec((world_->SimTime()).sec);
  state_msg.mutable_header()->mutable_stamp()->set_nsec(
      (world_->SimTime()).nsec);
  state_msg.mutable_position()->set_x(position.Z());
  state_msg.mutable_position()->set_y(velocity.Z());
  state_msg.mutable_position()->set_z(accel.Z());
  state_pub_->Publish(state_msg);
}

void GazeboHoverBallPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);
  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);

  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  state_pub_ = node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>(
      "~/" + namespace_ + "/" + state_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   state_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                state_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  load_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + load_sub_topic_,
                              &GazeboHoverBallPlugin::LoadCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                load_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   load_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);
}

void GazeboHoverBallPlugin::LoadCallback(GzWindSpeedMsgPtr& load_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
  payload_mass_ = load_msg->velocity().x() + ball_mass_;
  force_ = load_msg->velocity().z();
  bool is_paused = world_->IsPaused();
  if (!is_paused) {
    world_->SetPaused(true);
  }
  link_->AddForce(ignition::math::Vector3d(0, 0, force_));
  link_->GetInertial()->SetMass(payload_mass_);
  world_->SetPaused(is_paused);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboHoverBallPlugin);
}  // namespace gazebo
