/*
 * This node publics in a topic a boolean to give the trigger
 * to start the task
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

#define HZ 4

class Starter
{
public:
  Starter()
  :
  nh_("~"),
  max_runtime_(4.0)
  {
    t0_ = ros::Time::now();
    initParams();

    starter_pub_ = nh_.advertise<std_msgs::Bool>(starter_pub_topic_, 1);
  }

  void
  step()
  {
    std_msgs::Bool msg;

    msg.data = challengeStatusIsOk();
    starter_pub_.publish(msg);

    if ((ros::Time::now() - t0_).toSec() >= max_runtime_)
    {
      if (!msg.data)
      {
        ROS_ERROR("%s\n", "Robotech Challenge is not prepared to start!");
      }
      else
      {
        ROS_WARN("%s\n", "Task has started!");
      }
      
      ros::shutdown();
    }
  }

private:
  void
  initParams()
  {
    starter_pub_topic_ = "/robotech_vc_hub/race_start";

    nh_.param("pub_topic", starter_pub_topic_, starter_pub_topic_);
  }

  bool
  challengeStatusIsOk()
  {
    // CODE THIS!

    return true;
  }

  ros::NodeHandle nh_;
  
  ros::Publisher starter_pub_;    
  ros::Time t0_;

  double max_runtime_;
  std::string starter_pub_topic_;
};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "task_starter_node");

  Starter starter;

  ros::Rate loop_rate(HZ);
  while (ros::ok())
  {
    starter.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  exit(EXIT_SUCCESS);
}