#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <memory>

class SpeedChecker
{
public:
  SpeedChecker()
  :
  nh_("~"),
  vel_(NULL),
  limit_passed_(false)
  {
    initParams();

    vel_sub_ = nh_.subscribe(vel_topic_, 1, &SpeedChecker::velCallback, this);
  }

  void
  update()
  {
    if (vel_ == NULL)
      return;

    if (limit_passed_)
    {
      ROS_ERROR("[%s] robot passed the speed limit!\n", ros::this_node::getNamespace().c_str());
    }
    else
    {
      limit_passed_ = !linearOk(vel_->linear.x) || !angularOk(vel_->angular.z);
    }
  }

private:
  bool
  linearOk(double linear_vel)
  {
    return linear_vel <= max_linear_;
  }

  bool
  angularOk(double angular_vel)
  {
    return angular_vel <= max_angular_;
  }

  void
  velCallback(const geometry_msgs::Twist::ConstPtr & msg)
  {
    vel_ = std::make_shared<geometry_msgs::Twist>(*msg);
  }

  void
  initParams()
  {
    vel_topic_ = "/cmd_vel";
    max_linear_ = 0.7;    // m/s
    max_angular_ = 2.5;   // rad/s

    nh_.param("vel_topic", vel_topic_, vel_topic_);
    vel_topic_ = ros::this_node::getNamespace() + vel_topic_;

    nh_.param("max_linear", max_linear_, max_linear_);
    nh_.param("max_angular", max_angular_, max_angular_);
  }

  ros::NodeHandle nh_;
  ros::Subscriber vel_sub_;
  std::string vel_topic_;
  std::shared_ptr<geometry_msgs::Twist> vel_;
  double max_linear_, max_angular_;
  bool limit_passed_;
};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "speed_checker_node");

  SpeedChecker speed_checker;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    speed_checker.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}