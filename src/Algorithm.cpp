#include "hero_chassis_controller/Algorithm.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

//using namespace boost::accumulators;
//
//struct Algorithm::Data {
//  accumulator_set<double, features<tag::mean, tag::count>> acc;
//};

    Algorithm::~Algorithm()
    {
        sub_command_.shutdown();
    }



    bool Algorithm::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                         ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        controller_nh.getParam("wheel_track", wheel_track);
        controller_nh.getParam("wheel_base", wheel_base);

        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");

        pid_controller1_.init(ros::NodeHandle(controller_nh, "pid1"));
        pid_controller2_.init(ros::NodeHandle(controller_nh, "pid2"));
        pid_controller3_.init(ros::NodeHandle(controller_nh, "pid3"));
        pid_controller4_.init(ros::NodeHandle(controller_nh, "pid4"));

        last_time_ = ros::Time::now();
        controller_state_publisher_.reset(
                new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                        (controller_nh, "state", 1));

        sub_command_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Algorithm::SetCommandSpeed, this);
        odom_pub = controller_nh.advertise<nav_msgs::Odometry>("odom", 50);
        return true;
    }


    void Algorithm::update(const ros::Time &time, const ros::Duration& period)
    {
        now = time;

        Now_Vel_[0] = front_left_joint_.getVelocity();
        Now_Vel_[1] = front_right_joint_.getVelocity();
        Now_Vel_[2] = back_left_joint_.getVelocity();
        Now_Vel_[3] = back_right_joint_.getVelocity();

        ComRev();
        ComChassisVel();
        ComOdom();
        OdomPublish();

        error[0] = Vel_[0] - Now_Vel_[0];
        error[1] = Vel_[1] - Now_Vel_[1];
        error[2] = Vel_[2] - Now_Vel_[2];
        error[3] = Vel_[3] - Now_Vel_[3];
        commanded_effort[0] = pid_controller1_.computeCommand(error[0], period);
        commanded_effort[1] = pid_controller2_.computeCommand(error[1], period);
        commanded_effort[2] = pid_controller3_.computeCommand(error[2], period);
        commanded_effort[3] = pid_controller4_.computeCommand(error[3], period);
        front_left_joint_.setCommand(commanded_effort[0]);
        front_right_joint_.setCommand(commanded_effort[1]);
        back_left_joint_.setCommand(commanded_effort[2]);
        back_right_joint_.setCommand(commanded_effort[3]);

        StatePublish(period);
    }
    void Algorithm::SetCommandSpeed(const geometry_msgs::TwistConstPtr &msg)
    {
        Vx = msg->linear.x;
        Vy = msg->linear.y;
        Wa = msg->angular.z;
    }

    //各个轮子的期望转速
    void Algorithm::ComRev()
    {
        Vel_[0] = (Vx - Vy - (wheel_base + wheel_track) * Wa) / wheel_R;
        Vel_[1] = (Vx + Vy + (wheel_base + wheel_track) * Wa) / wheel_R;
        Vel_[2] = (Vx + Vy - (wheel_base + wheel_track) * Wa) / wheel_R;
        Vel_[3] = (Vx - Vy + (wheel_base + wheel_track) * Wa) / wheel_R;
    }

    //底盘的线速度和角速度
    void Algorithm::ComChassisVel()
    {
        Now_Vx = (Now_Vel_[0] + Now_Vel_[1] + Now_Vel_[2] + Now_Vel_[3]) * wheel_R / 4;
        Now_Vy = (-Now_Vel_[0] + Now_Vel_[1] + Now_Vel_[2] - Now_Vel_[3]) * wheel_R / 4;
        Now_Wa = (-Now_Vel_[0] + Now_Vel_[1] - Now_Vel_[2] + Now_Vel_[3]) * wheel_R / (4 * (wheel_base + wheel_track));
    }

    void Algorithm::StatePublish(const ros::Duration &period)
    {
        //十毫秒发送一次
        if (loop_count_ % 10 == 0) {
            if (controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = now;
                controller_state_publisher_->msg_.set_point = Vel_[0];
                controller_state_publisher_->msg_.process_value = Now_Vel_[0];
                controller_state_publisher_->msg_.error = error[0];
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = pid_controller1_.computeCommand(error[0], period);

                double dummy;
                bool antiwindup;
                pid_controller1_.getGains(controller_state_publisher_->msg_.p,
                                          controller_state_publisher_->msg_.i,
                                          controller_state_publisher_->msg_.d,
                                          controller_state_publisher_->msg_.i_clamp,
                                          dummy,
                                          antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;
        last_time_ = now;
    }

    void Algorithm::ComOdom()
    {
        dt = (now - last_time_).toSec();
        dx = (Now_Vx * cos(Wa) - Now_Vy * sin(Now_Wa)) * dt;
        dy = (Now_Vx * sin(Wa) + Now_Vy * cos(Now_Wa)) * dt;
        dth = Now_Wa * dt;

        x += dx;
        y += dy;
        th += dth;

        odom_quat = tf::createQuaternionMsgFromYaw(th);

        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
    }

    void Algorithm::OdomPublish()
    {
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = Now_Vx;
        odom.twist.twist.linear.y = Now_Vy;
        odom.twist.twist.angular.z = Now_Wa;
        //publish the message
        odom_pub.publish(odom);
    }



PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::Algorithm, controller_interface::ControllerBase)
} /* namespace */
