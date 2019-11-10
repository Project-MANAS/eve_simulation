/*
	Created on: 13-Sep-2018
	Author: naiveHobo
*/

#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>

#include "EvePlugin.hh"

#include <ros/ros.h>

using namespace gazebo;

EvePlugin::EvePlugin() {
  int argc = 0;
  char *argv = nullptr;
  ros::init(argc, &argv, "EvePlugin");

  direction_state_ = EvePlugin::FORWARD;
  front_left_wheel_radius_ = 0.281;
  front_right_wheel_radius_ = 0.281;
  back_left_wheel_radius_ = 0.281;
  back_right_wheel_radius_ = 0.281;

  target_linear_velocity_ = 0.0;
  target_angular_velocity_ = 0.0;
  target_steering_angle_ = 0.0;
}

EvePlugin::~EvePlugin() {
  update_connection_.reset();
}

void EvePlugin::manualControlCallback(const eve_msgs::ControlConstPtr &msg) {

  last_cmd_time_ = world_->SimTime();

  // Steering wheel command
  target_steering_angle_ = msg->steer;
  target_steering_angle_ = ignition::math::clamp(target_steering_angle_, steering_wheel_low_, steering_wheel_high_);

  // Brake command
  double brake = ignition::math::clamp(msg->brake, 0.0, 1.0);
  brake_percent_ = brake;

  // Throttle command
  double throttle = ignition::math::clamp(msg->throttle, 0.0, 1.0);
  throttle_percent_ = throttle;

  switch (msg->shift_gears) {
    case eve_msgs::Control::NEUTRAL: direction_state_ = EvePlugin::NEUTRAL;
      break;
    case eve_msgs::Control::FORWARD: direction_state_ = EvePlugin::FORWARD;
      break;
    case eve_msgs::Control::REVERSE: direction_state_ = EvePlugin::REVERSE;
      break;
    default: break;
  }
}

void EvePlugin::autonomousControlCallback(const geometry_msgs::Twist::ConstPtr &msg) {

  last_cmd_time_ = world_->SimTime();

  target_linear_velocity_ = msg->linear.x;
  target_angular_velocity_ = msg->angular.z;

  // if (convert_steer_) {
  //   if (target_angular_velocity_ == 0 || target_linear_velocity_ == 0)
  //     target_steering_angle_ = 0.0;
  //   else {
  //     double radius = target_linear_velocity_ / target_angular_velocity_;
  //     target_steering_angle_ = atan(wheel_base_length_ / radius);
  //   }
  // }
  // else
  //   target_steering_angle_ = target_angular_velocity_;
  //
  // target_steering_angle_ = ignition::math::clamp(target_steering_angle_, steering_wheel_low_, steering_wheel_high_);
}

void EvePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

  parent_ = model;
  sdf_ = sdf;
  world_ = parent_->GetWorld();

  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("frictionparent_", std::string("coneparent_"));

  std::string chassisLinkName = parent_->GetName() + "::" + sdf->Get<std::string>("base_link");
  chassis_link_ = parent_->GetLink(chassisLinkName);
  if (!chassis_link_) {
    ROS_ERROR("could not find base_link");
    return;
  }

  std::string steeringWheelJointName = parent_->GetName() + "::" + sdf->Get<std::string>("steering_wheel");
  steering_wheel_joint_ = parent_->GetJoint(steeringWheelJointName);
  if (!steering_wheel_joint_) {
    ROS_ERROR("could not find steering wheel joint");
    return;
  }

  std::string frontLeftWheelJointName = parent_->GetName() + "::" + sdf->Get<std::string>("front_left_wheel");
  front_left_wheel_joint_ = parent_->GetJoint(frontLeftWheelJointName);

  if (!front_left_wheel_joint_) {
    ROS_ERROR("could not find front left wheel joint");
    return;
  }

  std::string frontRightWheelJointName = parent_->GetName() + "::" + sdf->Get<std::string>("front_right_wheel");
  front_right_wheel_joint_ = parent_->GetJoint(frontRightWheelJointName);

  if (!front_right_wheel_joint_) {
    ROS_ERROR("could not find front right wheel joint");
    return;
  }

  std::string backLeftWheelJointName = parent_->GetName() + "::" + sdf->Get<std::string>("back_left_wheel");
  back_left_wheel_joint_ = parent_->GetJoint(backLeftWheelJointName);

  if (!back_left_wheel_joint_) {
    ROS_ERROR("could not find back left wheel joint");
    return;
  }

  std::string backRightWheelJointName = parent_->GetName() + "::" + sdf->Get<std::string>("back_right_wheel");
  back_right_wheel_joint_ = parent_->GetJoint(backRightWheelJointName);

  if (!back_right_wheel_joint_) {
    ROS_ERROR("could not find back right wheel joint");
    return;
  }

  std::string
      frontLeftWheelSteeringJointName = parent_->GetName() + "::" + sdf->Get<std::string>("front_left_wheel_steering");
  front_left_wheel_steering_joint_ = parent_->GetJoint(frontLeftWheelSteeringJointName);

  if (!front_left_wheel_steering_joint_) {
    ROS_ERROR("could not find front left steering joint");
    return;
  }

  std::string frontRightWheelSteeringJointName =
      parent_->GetName() + "::" + sdf->Get<std::string>("front_right_wheel_steering");
  front_right_wheel_steering_joint_ = parent_->GetJoint(frontRightWheelSteeringJointName);

  if (!front_right_wheel_steering_joint_) {
    ROS_ERROR("could not find front right steering joint");
    return;
  }

  getParam<bool>(autonomous_mode_, "autonomous_mode", false);
  getParam<bool>(convert_steer_, "convert_ang_vel_to_steer", true);
  getParam<std::string>(cmd_topic_, "cmd_topic", "/cmd_vel");
  getParam<std::string>(odom_topic_, "odom_topic", "/odom");
  getParam<std::string>(odom_frame_, "odom_frame", "odom");
  getParam<double>(odom_rate_, "odom_rate", 0.0);
  getParam<double>(front_torque_, "front_torque", 0.0);
  getParam<double>(back_torque_, "back_torque", 0.0);
  getParam<double>(front_brake_torque_, "front_brake_torque", 2000.0);
  getParam<double>(back_brake_torque_, "back_brake_torque", 2000.0);
  getParam<double>(max_speed_, "max_speed", 10.0);
  getParam<double>(max_steer_, "max_steer", 0.6);

  double pgain, dgain, igain, imax, imin;

  getParam<double>(pgain, "flwheel_steering_p_gain", 0.0);
  getParam<double>(igain, "flwheel_steering_i_gain", 0.0);
  getParam<double>(dgain, "flwheel_steering_d_gain", 0.0);
  front_left_wheel_steering_PID_.Init(pgain, dgain, igain);

  getParam<double>(pgain, "frwheel_steering_p_gain", 0.0);
  getParam<double>(igain, "frwheel_steering_i_gain", 0.0);
  getParam<double>(dgain, "frwheel_steering_d_gain", 0.0);
  front_right_wheel_steering_PID_.Init(pgain, dgain, igain);

  getParam<double>(pgain, "linear_velocity_p_gain", 0.0);
  getParam<double>(igain, "linear_velocity_i_gain", 0.0);
  getParam<double>(dgain, "linear_velocity_d_gain", 0.0);
  getParam<double>(imax, "linear_velocity_i_max", 0.0);
  getParam<double>(imin, "linear_velocity_i_min", 0.0);
  linear_velocity_PID_.Init(pgain, dgain, igain, imax, imin);

  getParam<double>(pgain, "angular_velocity_p_gain", 0.0);
  getParam<double>(igain, "angular_velocity_i_gain", 0.0);
  getParam<double>(dgain, "angular_velocity_d_gain", 0.0);
  getParam<double>(imax, "angular_velocity_i_max", 0.0);
  getParam<double>(imin, "angular_velocity_i_min", 0.0);
  angular_velocity_PID_.Init(pgain, dgain, igain, imax, imin);

  getParam<double>(wheel_joint_damping_, "wheel_joint_damping_coefficient", 0.0);

  ROS_INFO("AUTONOMOUS MODE IS %s", autonomous_mode_ ? "ENABLED" : "DISABLED");
  ROS_INFO("Listening on topic [%s] for commands", cmd_topic_.c_str());
  ROS_INFO("Advertising odometry information on topic [%s]", odom_topic_.c_str());
  ROS_INFO("Odometry frame set to [%s]", odom_frame_.c_str());

  // Assign callback based on autonomous mode value
  if (autonomous_mode_) {
    control_sub_ = nh_.subscribe(cmd_topic_, 10, &EvePlugin::autonomousControlCallback, this);
  } else {
    control_sub_ = nh_.subscribe(cmd_topic_, 10, &EvePlugin::manualControlCallback, this);
  }

  if (odom_rate_ > 0.0)
    odom_rate_ = 1.0 / odom_rate_;

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 1);

  UpdateSteeringWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  unsigned int id = 0;
  front_left_wheel_radius_ = CollisionRadius(front_left_wheel_joint_->GetChild()->GetCollision(id));
  front_right_wheel_radius_ = CollisionRadius(front_right_wheel_joint_->GetChild()->GetCollision(id));
  back_left_wheel_radius_ = CollisionRadius(back_left_wheel_joint_->GetChild()->GetCollision(id));
  back_right_wheel_radius_ = CollisionRadius(back_right_wheel_joint_->GetChild()->GetCollision(id));

  // Get initial joint friction and add it to braking friction
  front_left_joint_friction_ = front_left_wheel_joint_->GetParam("friction", 0);
  front_right_joint_friction_ = front_right_wheel_joint_->GetParam("friction", 0);
  back_left_joint_friction_ = back_left_wheel_joint_->GetParam("friction", 0);
  back_right_joint_friction_ = back_right_wheel_joint_->GetParam("friction", 0);

  // Add damping to joints to mimic friction
  front_left_wheel_joint_->SetDamping(0, wheel_joint_damping_);
  front_right_wheel_joint_->SetDamping(0, wheel_joint_damping_);
  back_left_wheel_joint_->SetDamping(0, wheel_joint_damping_);
  back_right_wheel_joint_->SetDamping(0, wheel_joint_damping_);

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  // first compute the positions of the 4 wheel centers
  // again assumes wheel link is child of joint and has only one collision
  ignition::math::Vector3d flCenterPos = front_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
  ignition::math::Vector3d frCenterPos = front_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
  ignition::math::Vector3d blCenterPos = back_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
  ignition::math::Vector3d brCenterPos = back_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  front_track_width_ = vec3.Length();
  vec3 = blCenterPos - brCenterPos;
  back_track_width_ = vec3.Length();

  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;

  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  wheel_base_length_ = vec3.Length();

  // Max force that can be applied to wheel steering joints
  double kMaxSteeringForceMagnitude = 5000;

  front_left_wheel_steering_PID_.SetCmdMax(kMaxSteeringForceMagnitude);
  front_left_wheel_steering_PID_.SetCmdMin(-kMaxSteeringForceMagnitude);

  front_right_wheel_steering_PID_.SetCmdMax(kMaxSteeringForceMagnitude);
  front_right_wheel_steering_PID_.SetCmdMin(-kMaxSteeringForceMagnitude);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&EvePlugin::Update, this));

  last_update_time_ = parent_->GetWorld()->SimTime();
}

void EvePlugin::Reset() {
  front_left_wheel_steering_PID_.Reset();
  front_right_wheel_steering_PID_.Reset();
  linear_velocity_PID_.Reset();
  angular_velocity_PID_.Reset();
  last_sim_time_ = 0;
  last_cmd_time_ = 0;
  direction_state_ = EvePlugin::FORWARD;
  front_left_wheel_steering_cmd_ = 0;
  front_right_wheel_steering_cmd_ = 0;
  throttle_percent_ = 0;
  brake_percent_ = 0;
  front_left_steering_angle_ = 0;
  front_right_steering_angle_ = 0;
  front_left_wheel_angular_velocity_ = 0;
  front_right_wheel_angular_velocity_ = 0;
  target_angular_velocity_ = 0.0;
  target_linear_velocity_ = 0.0;
  target_steering_angle_ = 0.0;
  last_update_time_ = parent_->GetWorld()->SimTime();
}

void EvePlugin::autonomousControl(double dt) {

  double velErr = current_linear_velocity_ - target_linear_velocity_;
  double velCmd = linear_velocity_PID_.Update(velErr, dt);

  double angErr = current_angular_velocity_ - target_angular_velocity_;
  double angCmd = angular_velocity_PID_.Update(angErr, dt);

  if (target_linear_velocity_ < 0.0)
    angCmd *= -1.0;

  target_steering_angle_ += angCmd;
  target_steering_angle_ = ignition::math::clamp(target_steering_angle_, -steer_limit_, steer_limit_);

  front_left_steering_angle_ = front_left_wheel_steering_joint_->Position(2);
  front_right_steering_angle_ = front_right_wheel_steering_joint_->Position(2);

  // Use when target steering angle is calculated for steering wheel and steering wheel to wheel turn mapping is figured out
  double tanSteer = tan(target_steering_angle_ * steering_ratio_);

  // Use when target steering angle is calculated using Ackermann Model and steer angle is calculated for center of front axle
  // double tanSteer = target_steering_angle_;

  front_left_wheel_steering_cmd_ = atan2(tanSteer, 1.0 - front_track_width_ / 2.0 / wheel_base_length_ * tanSteer);
  front_right_wheel_steering_cmd_ = atan2(tanSteer, 1.0 + front_track_width_ / 2.0 / wheel_base_length_ * tanSteer);

  double flwsError = front_left_steering_angle_ - front_left_wheel_steering_cmd_;
  double flwsCmd = front_left_wheel_steering_PID_.Update(flwsError, dt);
  front_left_wheel_steering_joint_->SetForce(0, flwsCmd);

  double frwsError = front_right_steering_angle_ - front_right_wheel_steering_cmd_;
  double frwsCmd = front_right_wheel_steering_PID_.Update(frwsError, dt);
  front_right_wheel_steering_joint_->SetForce(0, frwsCmd);

  front_left_wheel_joint_->SetParam("friction", 0, front_left_joint_friction_);
  front_right_wheel_joint_->SetParam("friction", 0, front_right_joint_friction_);
  back_left_wheel_joint_->SetParam("friction", 0, back_left_joint_friction_);
  back_right_wheel_joint_->SetParam("friction", 0, back_right_joint_friction_);

  back_left_wheel_joint_->SetForce(0, velCmd);
  back_right_wheel_joint_->SetForce(0, velCmd);

  // reset if last command is more than x sec ago
  common::Time curTime = world_->SimTime();
  if ((curTime - last_cmd_time_).Double() > 0.15) {
    target_angular_velocity_ = 0.0;
    target_linear_velocity_ = 0.0;
    target_steering_angle_ = 0.0;
  }
}

void EvePlugin::Update() {

  std::lock_guard<std::mutex> lock(mutex_);

  common::Time curTime = world_->SimTime();
  double dt = (curTime - last_sim_time_).Double();
  if (dt < 0) {
    Reset();
    return;
  } else if (ignition::math::equal(dt, 0.0)) {
    return;
  }

  updateOdom();

  double seconds_since_last_update = (curTime - last_update_time_).Double();
  if (seconds_since_last_update > odom_rate_) {
    publishOdometry();
    last_update_time_ += common::Time(odom_rate_);
  }

  if (autonomous_mode_) {
    autonomousControl(dt);
    return;
  }

  front_left_steering_angle_ = front_left_wheel_steering_joint_->Position(2);
  front_right_steering_angle_ = front_right_wheel_steering_joint_->Position(2);

  front_left_wheel_angular_velocity_ = front_left_wheel_joint_->GetVelocity(0);
  front_right_wheel_angular_velocity_ = front_right_wheel_joint_->GetVelocity(0);
  back_left_wheel_angular_velocity_ = back_left_wheel_joint_->GetVelocity(0);
  back_right_wheel_angular_velocity_ = back_right_wheel_joint_->GetVelocity(0);

  // Get linear velocity at base_link centre of gravity
  chassis_linear_velocity_ = chassis_link_->WorldLinearVel();

  // PID (position) steering
  target_steering_angle_ = ignition::math::clamp(target_steering_angle_, -steer_limit_, steer_limit_);

  // PID (position) steering joints based on steering position
  // Ackermann steering geometry
  double tanSteer = tan(target_steering_angle_ * steering_ratio_);
  front_left_wheel_steering_cmd_ = atan2(tanSteer, 1.0 - front_track_width_ / 2.0 / wheel_base_length_ * tanSteer);
  front_right_wheel_steering_cmd_ = atan2(tanSteer, 1.0 + front_track_width_ / 2.0 / wheel_base_length_ * tanSteer);

  double flwsError = front_left_steering_angle_ - front_left_wheel_steering_cmd_;
  double flwsCmd = front_left_wheel_steering_PID_.Update(flwsError, dt);
  front_left_wheel_steering_joint_->SetForce(0, flwsCmd);

  double frwsError = front_right_steering_angle_ - front_right_wheel_steering_cmd_;
  double frwsCmd = front_right_wheel_steering_PID_.Update(frwsError, dt);
  front_right_wheel_steering_joint_->SetForce(0, frwsCmd);

  double gasPercent = throttle_percent_;
  double gasMultiplier = GasTorqueMultiplier();

  double blGasTorque = 0, brGasTorque = 0;

  if (fabs(back_left_wheel_angular_velocity_ * back_left_wheel_radius_) < max_speed_
      && fabs(back_right_wheel_angular_velocity_ * back_right_wheel_radius_) < max_speed_) {
    blGasTorque = gasPercent * front_torque_ * gasMultiplier;
    brGasTorque = gasPercent * front_torque_ * gasMultiplier;
  }

  double brakePercent = ignition::math::clamp(brake_percent_, 0.0, 1.0);

  front_left_wheel_joint_->SetParam("friction", 0, front_left_joint_friction_ + brakePercent * front_brake_torque_);
  front_right_wheel_joint_->SetParam("friction", 0, front_right_joint_friction_ + brakePercent * front_brake_torque_);
  back_left_wheel_joint_->SetParam("friction", 0, back_left_joint_friction_ + brakePercent * back_brake_torque_);
  back_right_wheel_joint_->SetParam("friction", 0, back_right_joint_friction_ + brakePercent * back_brake_torque_);

  // Rear Wheel Drive
  back_left_wheel_joint_->SetForce(0, blGasTorque);
  back_right_wheel_joint_->SetForce(0, brGasTorque);

  // reset if last command is more than x sec ago
  if ((curTime - last_cmd_time_).Double() > 0.15) {
    throttle_percent_ = 0.0;
    brake_percent_ = 0.0;
  }

  last_sim_time_ = curTime;
}

void EvePlugin::updateOdom() {
  tf::Quaternion qt;
  tf::Vector3 vt;

  ignition::math::Pose3d pose = parent_->WorldPose();

  qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  // get velocity in odom frame
  ignition::math::Vector3d linear;

  linear = parent_->WorldLinearVel();
  current_angular_velocity_ = parent_->WorldAngularVel().Z();
  odom_.twist.twist.angular.z = current_angular_velocity_;

  // convert velocity to child_frame_id (aka base_link)
  double yaw = pose.Rot().Yaw();
  current_linear_velocity_ = cos(yaw) * linear.X() + sin(yaw) * linear.Y();
  odom_.twist.twist.linear.x = current_linear_velocity_;
  odom_.twist.twist.linear.y = cos(yaw) * linear.Y() - sin(yaw) * linear.X();

  odom_tf_ = tf::Transform(qt, vt);
}

void EvePlugin::publishOdometry() {

  ros::Time current_time = ros::Time::now();

  transform_broadcaster_.sendTransform(tf::StampedTransform(odom_tf_, current_time, odom_frame_, "base_link"));

  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame_;
  odom_.child_frame_id = "base_link";

  odom_pub_.publish(odom_);

//  ROS_INFO("Published odometry message");
}

void EvePlugin::UpdateSteeringWheelRatio() {
  // The total range the steering wheel can rotate
  steering_wheel_high_ = 12.5664;
  steering_wheel_low_ = -12.5664;
  double steeringWheelRange = steering_wheel_high_ - steering_wheel_low_;
  double high = 0.680678;
  high = std::min(high, max_steer_);
  double low = -0.523599;
  low = std::max(low, -max_steer_);
  double tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  steering_ratio_ = tireAngleRange / steeringWheelRange;

  steer_limit_ = max_steer_ / steering_ratio_;
}

double EvePlugin::CollisionRadius(physics::CollisionPtr _coll) {
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    physics::CylinderShape *cyl = dynamic_cast<physics::CylinderShape *>(_coll->GetShape().get());
    return cyl->GetRadius();
  } else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE)) {
    physics::SphereShape *sph = dynamic_cast<physics::SphereShape *>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

double EvePlugin::GasTorqueMultiplier() {
  if (direction_state_ == EvePlugin::FORWARD)
    return 1.0;
  else if (direction_state_ == EvePlugin::REVERSE)
    return -1.0;
}

GZ_REGISTER_MODEL_PLUGIN(EvePlugin)
