#include "rotors_gazebo_plugins/gazebo_cable_plugin.h"
#include <rotors_gazebo_plugins/common.h>

#include <iostream>

#include "ConnectGazeboToRosTopic.pb.h"
#include "Vector3dStamped.pb.h"

namespace gazebo {
void GazeboCablePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_cable_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();
  
  if (_sdf->HasElement("linkName")) {
    std::string link_name = _sdf->GetElement("linkName")->Get<std::string>();
    link_ = model_->GetLink(link_name);
    if (link_ == NULL) {
      gzthrow("[gazebo_cable_plugin] Couldn't find specified link \""
              << link_name << "\".");
    }
  } else {
    gzthrow("[gazebo_cable_plugin] Please specify a linkName.\n");
  }

  if (_sdf->HasElement("parentLinkName")) {
    std::string link_name = _sdf->GetElement("parentLinkName")->Get<std::string>();
    parent_link_ = model_->GetLink(link_name);
    if (parent_link_ == NULL) {
      gzthrow("[gazebo_cable_plugin] Couldn't find specified link \""
              << link_name << "\".");
    }
  } else {
    gzthrow("[gazebo_cable_plugin] Please specify a parentLinkName.\n");
  }

  getSdfParam<double>(_sdf, "ballMass", ball_mass_, ball_mass_);
  getSdfParam<double>(_sdf, "cableLength", cable_length_, cable_length_);
  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, pose_pub_topic_);

  mav_mass_ = parent_link_->GetInertial()->Mass();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboCablePlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboCablePlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  ignition::math::Pose3d W_pose_W_C = link_->WorldCoGPose();
  ignition::math::Vector3d W_vel_W_C = link_->WorldLinearVel();
  ignition::math::Pose3d W_pose_W_P = parent_link_->WorldCoGPose();
  ignition::math::Vector3d W_vel_W_P = parent_link_->WorldLinearVel();

  ignition::math::Vector3d W_pos_P_C = (W_pose_W_C - W_pose_W_P).Pos();
  ignition::math::Vector3d W_unit_pos_P_C(W_pos_P_C);
  W_unit_pos_P_C.Normalize();

  ignition::math::Vector3d W_angular_momentum_P_C = -parent_link_->WorldAngularMomentum();
  ignition::math::Vector3d W_angular_velocity_P_C = parent_link_->WorldInertiaMatrix().Inverse() * W_angular_momentum_P_C;
  ignition::math::Vector3d W_vel_P_C = W_angular_velocity_P_C.Cross(W_unit_pos_P_C);

  if (W_pos_P_C.Length() >= (cable_length_- 1e-3)) {
    //calculation of thrust
    ignition::math::Vector3d f = parent_link_->WorldForce() - (ball_mass_ + mav_mass_) * world_->Gravity();

    //(m_q + m_l)(v_l_dot + g e3) = (q f R e3 - m_q l (q_dot q_dot)) q
    ignition::math::Vector3d W_force_W_C = (W_unit_pos_P_C.Dot(f) - ball_mass_ * cable_length_ * (W_vel_P_C.Dot(W_vel_P_C))) * W_unit_pos_P_C;
    ignition::math::Vector3d W_accel_W_C = (W_force_W_C / (ball_mass_ + mav_mass_)) + world_->Gravity();
    ignition::math::Vector3d tension = ball_mass_ * (W_accel_W_C - world_->Gravity());

    ignition::math::Vector3d W_fixed_pos_P_C = W_pos_P_C * cable_length_ / W_pos_P_C.Length();
    ignition::math::Quaterniond W_fixed_rot_P_C = (W_pose_W_C - W_pose_W_P).Rot();
    ignition::math::Pose3d W_fixed_pose_P_C(W_fixed_pos_P_C, W_fixed_rot_P_C);
    ignition::math::Vector3d W_fixed_vel_W_C = W_vel_W_C - (W_vel_W_C - W_vel_W_P).Dot(W_unit_pos_P_C) * W_unit_pos_P_C;

    bool is_paused = world_->IsPaused();
    if (!is_paused) {
      world_->SetPaused(true);
    }
    link_->SetRelativePose(W_fixed_pose_P_C);

    // set link_ velocity to desired twist
    link_->SetLinearVel(W_fixed_vel_W_C);
    // link_->SetAngularVel(ignition::math::Vector3d(0,0,0));

    link_->AddForce(tension);
    parent_link_->AddForce(-tension);
    world_->SetPaused(is_paused);
  }

  gz_geometry_msgs::Vector3dStamped pose_msg;
  pose_msg.mutable_header()->set_frame_id("world");
  pose_msg.mutable_header()->mutable_stamp()->set_sec((world_->SimTime()).sec);
  pose_msg.mutable_header()->mutable_stamp()->set_nsec((world_->SimTime()).nsec);
  pose_msg.mutable_position()->set_x(W_pose_W_C.X());
  pose_msg.mutable_position()->set_y(W_pose_W_C.Y());
  pose_msg.mutable_position()->set_z(W_pose_W_C.Z());
  pose_pub_->Publish(pose_msg);
}

void GazeboCablePlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  pose_pub_ = node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>(
      "~/" + namespace_ + "/" + pose_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCablePlugin);
}  // namespace gazebo

// std::cout << "******\n\n"
//           << tension << "\n*" << W_pos_P_C.Length() << "\n*" << world_->Gravity() << "\n\n******" << std::endl;
// for (auto link : model_->GetLinks())
// {
//   std::cout << link->GetName() << "\n";
// }

// thrust get from motorForcePubTopic - expose topic

//   ignition::math::Vector3d C_linear_velocity_W_C =
//   link_->RelativeLinearVel(); ignition::math::Vector3d
//   C_angular_velocity_W_C = link_->RelativeAngularVel();

//   ignition::math::Vector3d gazebo_linear_velocity = C_linear_velocity_W_C;
//   ignition::math::Vector3d gazebo_angular_velocity =
//   C_angular_velocity_W_C; ignition::math::Pose3d gazebo_pose = W_pose_W_C;

//   //   ignition::math::Vector3d thrust;

//   if (parent_frame_id_ != kDefaultParentFrameId) {
//     ignition::math::Pose3d W_pose_W_P = parent_link_->WorldPose();
//     ignition::math::Vector3d P_linear_velocity_W_P =
//         parent_link_->RelativeLinearVel();
//     ignition::math::Vector3d P_angular_velocitfy_W_P =
//         parent_link_->RelativeAngularVel();
//     // thrust = parent_link_->RelativeLinearAccel();

//     ignition::math::Pose3d P_pose_P_C = W_pose_W_C - W_pose_W_P;
//     ignition::math::Vector3d P_linear_velocity_P_C;

//     P_linear_velocity_P_C = -P_angular_velocity_W_P.Cross(P_pose_P_C.Pos())
//     +
//                             P_pose_P_C.Rot() * C_linear_velocity_W_C -
//                             P_linear_velocity_W_P;

//     // C_angular_velocity_W_C = 0; // assume child does not rotate on its
//     own gazebo_angular_velocity =
//         P_pose_P_C.Rot() * C_angular_velocity_W_C - P_angular_velocity_W_P;
//     gazebo_linear_velocity = P_linear_velocity_P_C;
//     gazebo_pose = P_pose_P_C;
//   }