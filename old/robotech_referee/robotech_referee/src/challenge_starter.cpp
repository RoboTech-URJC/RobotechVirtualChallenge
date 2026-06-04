/*
 * This node publics in a topic named '/robotech_vc_hub/match_info' the
 * information that the contestants needs to know before to start the challenge
 */

#include <ros/ros.h>
#include "robotech_referee_msgs/MatchInfo.h"
#include "robotech_referee_msgs/MatchInit.h"

#include <string>
#include <vector>

#define HZ 4

typedef struct MatchInfoType MatchInfoType;
struct MatchInfoType {
  std::string team_name;
  std::string opponent_color;
  std::string rol;
};

class Starter
{
public:
  Starter()
  :
  nh_("~"),
  max_runtime_(8.0)
  {
    initParams();
    t0_ = ros::Time::now();
    init_pub_ = nh_.advertise<robotech_referee_msgs::MatchInit>("/robotech_vc_hub/match_info", 1);
  }

  void
  step()
  {
    std::vector<robotech_referee_msgs::MatchInfo> v;
    robotech_referee_msgs::MatchInfo info;
    robotech_referee_msgs::MatchInit match_init;

    info.team_name = team1_info_.team_name;
    info.opponent_color = team1_info_.opponent_color;
    info.rol = team1_info_.rol;

    v.push_back(info);

    info.team_name = team2_info_.team_name;
    info.opponent_color = team2_info_.opponent_color;
    info.rol = team2_info_.rol;

    v.push_back(info);

    match_init.info = v;
    
    init_pub_.publish(match_init);
  }

  bool
  ok()
  {
    return (ros::Time::now() - t0_).toSec() < max_runtime_;
  }

private:
  void initParams()
  {
    team1_info_.team_name = "algo1";
    team1_info_.opponent_color = "red";
    team1_info_.rol = "crew_member";

    team2_info_.team_name = "algo2";
    team2_info_.opponent_color = "purple";
    team2_info_.rol = "impostor";

    nh_.param("team1_name", team1_info_.team_name, team1_info_.team_name);
    nh_.param("opponent1_color", team1_info_.opponent_color, team1_info_.opponent_color);
    nh_.param("team1_rol", team1_info_.rol, team1_info_.rol);

    nh_.param("team2_name", team2_info_.team_name, team2_info_.team_name);
    nh_.param("opponent2_color", team2_info_.opponent_color, team2_info_.opponent_color);
    nh_.param("team2_rol", team2_info_.rol, team2_info_.rol);
  }

  ros::NodeHandle nh_;
  
  ros::Publisher init_pub_;    
  ros::Time t0_;

  double max_runtime_;

  MatchInfoType team1_info_, team2_info_;
};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "challenge_starter_node");

  Starter starter;

  ros::Rate loop_rate(HZ);
  while (starter.ok() && ros::ok())
  {
    starter.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  exit(EXIT_SUCCESS);
}