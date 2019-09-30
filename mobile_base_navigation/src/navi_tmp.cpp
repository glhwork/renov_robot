#include "costmap_2d/costmap_2d_ros.h"
#include "nav_core/base_global_planner.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"

class MobileBasePlanner {
 public:
  MobileBasePlanner(ros::NodeHandle nh, ros::NodeHandle nh_private,
                    std::string& name,
                    costmap_2d::Costmap2DROS* costmap_2d_ros);
  virtual ~MobileBasePlanner() {}
  void ParamInit(ros::NodeHandle& nh_private);
  void PlanCallback(const geometry_msgs::PoseStamped& goal_pose);

 private:
  std::string path_pub_topic_;
  std::string goal_sub_topic_;
  std::string path_frame_id_;

  nav_core::BaseGlobalPlanner base_global_planner_;
  ros::Publisher path_pub_;
  ros::Subscriber goal_sub_;
};

MobileBasePlanner::MobileBasePlanner(ros::NodeHandle nh,
                                     ros::NodeHandle nh_private,
                                     std::string& name,
                                     costmap_2d::Costmap2DROS* costmap_2d_ros) {
  base_global_planner_.initialize(name, costmap_2d_ros);
  ParamInit(nh_private);
  path_pub_ = nh.advertise<nav_msgs::Path>(path_pub_topic_, 10);
  goal_sub_ =
      nh.subscribe(goal_sub_topic_, 10, &MobileBasePlanner::PlanCallback, this);
}

void MobileBasePlanner::ParamInit(ros::NodeHandle& nh_private) {
  nh_private.param("path_pub_topic", path_pub_topic_,
                   std::string("mobile_base_path"));
  nh_private.param("goal_sub_topic", goal_sub_topic_,
                   std::string("mobile_base_goal"));
  nh_private.param("path_frame_id", path_frame_id_, std::string("map"));
}

void MobileBasePlanner::PlanCallback(
    const geometry_msgs::PoseStamped& goal_pose) {
  tf2_ros::TransformListener tl;
  geometry_msgs::PoseStamped start;

  std::vector<geometry_msgs::PoseStamped> pose_vec;
  base_global_planner_.makePlan(start, goal_pose, pose_vec);

  nav_msgs::Path path;
  path.header.frame_id = path_frame_id_;
  path.header.stamp = pose_vec[0].header.stamp;
  for (size_t i = 0; i < pose_vec.size(); i++) {
    geometry_msgs::PoseStamped pose_stamped_tmp;
    pose_stamped_tmp.header = pose_vec[i].header;
    pose_stamped_tmp.pose = pose_vec[i].pose;

    path.poses.push_back(pose_stamped_tmp);
  }
  path_pub_.publish(path);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mobile_base_planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  std::string costmap_name = "mobile_base_costmap";
  costmap_2d::Costmap2DROS lcr(costmap_name, buffer);

  ros::spin();

  return 0;
}