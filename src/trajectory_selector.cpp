/*
  Autoware trajectory publisher node in carla simulatior, KIAPI and K-city
  map name : Town04, KIAPI, K-city
  node name : trajectory_selector_node
  e-mail : yunsh3594@gmail.com 
  made by. Seunghoe Yun
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lane_change_check.hpp>
#include <mutex>

class TrajectorySelector : public rclcpp::Node
{
public:
  TrajectorySelector() : Node("trajectory_selector")
  {
    lccMsg.changeflag = false;
    TR_route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>("/planning/trajectory_router/route", rclcpp::QoS{1}.transient_local(), 
                                                          std::bind(&TrajectorySelector::TRroutecallback, this, std::placeholders::_1));
    LC_route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>("/planning/obstacle_lane_change/route", rclcpp::QoS{1}.transient_local(), 
                                                          std::bind(&TrajectorySelector::LCroutecallback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectorySelector::callbackPose, this, std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", 
                                                                            rclcpp::QoS{1}.transient_local());
    lane_change_check_sub_= create_subscription<autoware_planning_msgs::msg::LaneChangeCheck>("/planning/lane_change_check", rclcpp::QoS{1}, std::bind(&TrajectorySelector::callbackLaneChangeCheck, this, std::placeholders::_1));
  }
  autoware_planning_msgs::msg::LaneletRoute TR_route_msg_;
  autoware_planning_msgs::msg::LaneletRoute LC_route_msg_;
  autoware_planning_msgs::msg::LaneChangeCheck lccMsg;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::ConstSharedPtr TR_route_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::ConstSharedPtr LC_route_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneChangeCheck>::SharedPtr lane_change_check_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
private:
  void TRroutecallback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
  {
    TR_route_msg_ = *msg;
    TR_route_msg_.header.stamp = this->get_clock()->now();
    TR_route_msg_.header.frame_id = "map";
    std::cout << "trajectory route pub" << std::endl;
    route_pub_->publish(TR_route_msg_);
  }

  void LCroutecallback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
  {
    // std::cout << msg->header.stamp.sec << std::endl;
    LC_route_msg_ = *msg;
    LC_route_msg_.header.stamp = this->get_clock()->now();
    LC_route_msg_.header.frame_id = "map";
    std::cout << "obsatcle lane change pub" << std::endl;
    route_pub_->publish(LC_route_msg_);
  }

  // lane change 여부를 판단하는 토픽을 subscribe한다. 
  // lock을 이용해야 한다.
  void callbackLaneChangeCheck(const autoware_planning_msgs::msg::LaneChangeCheck msg)
  {
    lccMsg = msg;
    std::cout << "Get LaneChangeSignal : " << msg.changeflag << std::endl;
  }

  // pose를 받아서 field를 변경해야 하나? 
  void callbackPose(const geometry_msgs::msg::PoseStamped msg)
  {
    if(lccMsg.changeflag)
    {
      TR_route_msg_.header.stamp = this->get_clock()->now();
      TR_route_msg_.header.frame_id = "map";
      TR_route_msg_.start_pose = msg.pose;
      std::cout << "2-trajectory route pub" << std::endl;
      route_pub_->publish(TR_route_msg_);
      lccMsg.changeflag = false;
    }
  }
};

int main(int argc, char **argv)
{    
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectorySelector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}