/*
    Created on: 13-Sep-2018
    Author: naiveHobo
*/

#ifndef _EVE_PLUGIN_HH_
#define _EVE_PLUGIN_HH_

#include <memory>

#include <ignition/math/Vector3.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <eve_msgs/Control.h>
#include <nav_msgs/Odometry.h>

namespace gazebo {

// A model plugin for Eve
class EvePlugin : public ModelPlugin {

 private:

  enum DirectionType {
    REVERSE = -1,
    NEUTRAL = 0,
    FORWARD = 1
  };

  ros::NodeHandle nh_;

  ros::Subscriber control_sub_;
  ros::Publisher odom_pub_;

  tf::TransformBroadcaster transform_broadcaster_;
  tf::Transform odom_tf_;

  std::string cmd_topic_;
  std::string odom_topic_;
  std::string odom_frame_;

  double odom_rate_;

  nav_msgs::Odometry odom_;

  physics::WorldPtr world_;
  physics::ModelPtr parent_;
  sdf::ElementPtr sdf_;

  event::ConnectionPtr update_connection_;

  physics::LinkPtr chassis_link_;

  // Wheel Joints
  physics::JointPtr front_left_wheel_joint_;
  physics::JointPtr front_right_wheel_joint_;
  physics::JointPtr back_left_wheel_joint_;
  physics::JointPtr back_right_wheel_joint_;

  // Wheel Steering Joints
  physics::JointPtr front_left_wheel_steering_joint_;
  physics::JointPtr front_right_wheel_steering_joint_;

  // Steering Wheel Joint
  physics::JointPtr steering_wheel_joint_;

  // PID control for velocity
  common::PID linear_velocity_PID_;
  common::PID angular_velocity_PID_;

  // PID control for wheel steering joints
  common::PID front_left_wheel_steering_PID_;
  common::PID front_right_wheel_steering_PID_;

  // Last sim time received
  common::Time last_sim_time_;

  // Last command time
  common::Time last_cmd_time_;

  // Last odometry update time
  common::Time last_update_time_;

  // Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
  DirectionType direction_state_;

  // Boolean flag to decide if Eve is in Autonomous/Manual mode
  bool autonomous_mode_;

  // Boolean flag to convert angular.z in Twist messages to steering angle
  bool convert_steer_;

  // Max torque that can be applied to wheels
  double front_torque_ = 0;
  double back_torque_ = 0;

  // Max torque that can be applied by brakes
  double front_brake_torque_ = 0;
  double back_brake_torque_ = 0;

  // Max speed (m/s) of the car
  double max_speed_ = 0;

  // Max steering angle
  double max_steer_ = 0;

  // Gas pedal position in percentage. 1.0 = Fully accelerated.
  double throttle_percent_ = 0;

  // Brake pedal position in percentage. 1.0 = Full brake.
  double brake_percent_ = 0;

  // Angle ratio between the steering wheel and the front wheels
  double steering_ratio_ = 0;

  // Limit on steering wheel rotation
  double steer_limit_ = 0;

  // Min-Max range of hand steering wheel
  double steering_wheel_high_ = 0;
  double steering_wheel_low_ = 0;

  // Front wheel desired steering angle (radians)
  double front_left_wheel_steering_cmd_ = 0;
  double front_right_wheel_steering_cmd_ = 0;

  // Steering wheel desired angle (radians)
  double target_steering_angle_ = 0;

  // Wheel radius
  double front_left_wheel_radius_ = 0;
  double front_right_wheel_radius_ = 0;
  double back_left_wheel_radius_ = 0;
  double back_right_wheel_radius_ = 0;

  // Wheel joint friction
  double front_left_joint_friction_ = 0;
  double front_right_joint_friction_ = 0;
  double back_left_joint_friction_ = 0;
  double back_right_joint_friction_ = 0;

  // Wheel joint damping coefficient
  double wheel_joint_damping_ = 0;

  // Distance between front and rear axles
  double wheel_base_length_ = 0;

  // Distance between front left and right wheels
  double front_track_width_ = 0;

  // Distance between rear left and right wheels
  double back_track_width_ = 0;

  // Steering angle of front wheels at last update (radians)
  double front_left_steering_angle_ = 0;
  double front_right_steering_angle_ = 0;

  // Linear velocity of base_link in world frame at last update (m/s)
  ignition::math::Vector3d chassis_linear_velocity_;

  // Angular velocity of base_link in world frame at last update (rad/s)
  ignition::math::Vector3d chassis_angular_velocity_;

  // Current linear velocity of base_link (m/s)
  double current_linear_velocity_;

  // Current angular velocity of base_link (rad/s)
  double current_angular_velocity_;

  // Target linear velocity of base_link (m/s)
  double target_linear_velocity_;

  // Target angular velocity of base_link (rad/s)
  double target_angular_velocity_;

  // Angular velocity of wheels at last update (rad/s)
  double front_left_wheel_angular_velocity_ = 0;
  double front_right_wheel_angular_velocity_ = 0;
  double back_left_wheel_angular_velocity_ = 0;
  double back_right_wheel_angular_velocity_ = 0;

  // Mutex to protect updates
  std::mutex mutex_;

  // brief ROS subscriber callback
  void manualControlCallback(const eve_msgs::ControlConstPtr &msg);
  void autonomousControlCallback(const geometry_msgs::Twist::ConstPtr &msg);

  // calculates torque _lastSimTime = curTime;to be applied on wheels and steering to achieve given Twist command
  void autonomousControl(double dt);

  // Update on every time step
  void Update();

  // Update steering wheel to front left/right wheel ratio
  void UpdateSteeringWheelRatio();

  // Get the radius of a collision
  double CollisionRadius(physics::CollisionPtr _collision);

  // Get the multiplier that is determined based on the direction state of the vehicle.
  double GasTorqueMultiplier();

  void publishOdometry();

  void updateOdom();

  // Get parameters for plugin
  template<typename T>
  void getParam(T &obj, std::string paramName, T defaultValue) {
    if (sdf_->HasElement(paramName))
      obj = sdf_->Get<T>(paramName);
    else
      obj = defaultValue;
  }

 public:

  // Constructor.
  EvePlugin();

  // brief Destructor.
  ~EvePlugin() override;

  // Documentation Inherited
  void Reset() override;

  // Load the controller.
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;

};
}

#endif
