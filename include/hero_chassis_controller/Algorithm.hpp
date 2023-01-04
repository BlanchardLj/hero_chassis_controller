#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#define wheel_R   0.07625
namespace hero_chassis_controller {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
 public:
  /*!
   * Constructor.
   */
  Algorithm() = default;

  /*!
   * Destructor.
   */
  ~Algorithm() override;

  /*!
   * Add new measurement data.
   * @param data the new data.
   */

  //void addData(const double data);
  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration& period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

  control_toolbox::Pid pid_controller1_, pid_controller2_, pid_controller3_, pid_controller4_;

  ros::Subscriber sub_command_;
  ros::Publisher odom_pub;

  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat;
  geometry_msgs::TransformStamped odom_trans;

 private:
  int loop_count_;
  double Vel_[4]{0.0, 0.0, 0.0, 0.0}; //期望的转速
  double Now_Vel_[4]{0.0, 0.0, 0.0, 0.0};//当前的转速
  double error[4]{0.0, 0.0, 0.0, 0.0};
  double commanded_effort[4]{0.0, 0.0, 0.0, 0.0};
  double Vx, Vy, Wa;//期望的底盘速度
  double Now_Vx, Now_Vy, Now_Wa;//实际的底盘速度
  double wheel_track,wheel_base;
  double dt, dx, dy, dth;
  double x = 0.0, y = 0.0, th = 0.0;
  ros::Time last_time_;
  ros::Time now;

  std::unique_ptr<
            realtime_tools::RealtimePublisher<
                    control_msgs::JointControllerState> > controller_state_publisher_ ;

  void SetCommandSpeed(const geometry_msgs::TwistConstPtr &msg);
  void ComRev();
  void ComChassisVel();
  void ComOdom();
  void OdomPublish();
  void StatePublish(const ros::Duration &period);
};

} /* namespace */

#endif