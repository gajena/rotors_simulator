#include "rotors_gazebo_plugins/gazebo_cable_plugin.h"

#include <rotors_gazebo_plugins/common.h>

#include <chrono>
#include <iostream>

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "PointStamped.pb.h"
#include "TransformStamped.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "Vector3dStamped.pb.h"

namespace gazebo {

GazeboCablePlugin::~GazeboCablePlugin() {}

void GazeboCablePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_cable_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();
  for (auto link : model_->GetLinks()) {
    std::cout << link->GetName() << "\n";
  }

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_cable_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_cable_plugin] Couldn't find specified link \""
            << link_name_ << "\".");

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_cable_plugin] Please specify a jointName.\n";
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_cable_plugin] Couldn't find specified joint \""
            << joint_name_ << "\".");

  getSdfParam<double>(_sdf, "ballRadius", ball_radius_, ball_radius_);
  getSdfParam<double>(_sdf, "ballWeight", ball_weight_, ball_weight_);
  getSdfParam<double>(_sdf, "cableLength", cable_length_, cable_length_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_,
                           parent_frame_id_);
  getSdfParam<std::string>(_sdf, "childFrameId", child_frame_id_,
                           child_frame_id_);
  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, pose_pub_topic_);

  std::cout << "[CABLE]\n"
            << parent_frame_id_ << "\n"
            << child_frame_id_ << "\n";

  parent_link_ = world_->EntityByName(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \""
            << parent_frame_id_ << "\".");
  }

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
  ignition::math::Vector3d W_unit_pos_W_C = W_pose_W_C.Pos().Normalize();

  ignition::math::Matrix3d W_inertia_matrix_W_C = link_->WorldInertiaMatrix();
  ignition::math::Vector3d W_angular_momentum_W_C =
      link_->WorldAngularMomentum();
  ignition::math::Vector3d W_angular_velocity_W_C =
      W_inertia_matrix_W_C.Inverse() * W_angular_momentum_W_C;

  ignition::math::Vector3d W_vel_W_C =
      W_angular_velocity_W_C.Cross(W_unit_pos_W_C);

  std::cout << "******\n\n" << joint_->GetForce(0) << "\n\n******" << std::endl;

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

  //   ignition::math::Vector3d tension(0.0, 0.0, 0.0);
  //   if (gazebo_pose.norm() >= cable_length_) {
  //   }
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

  broadcast_transform_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStampedWithFrameIds>(
          "~/" + kBroadcastTransformSubtopic, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCablePlugin);

}  // namespace gazebo