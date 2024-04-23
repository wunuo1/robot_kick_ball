#ifndef KICK_BALL_H_
#define KICK_BALL_H_

#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"

class KickBallNode : public rclcpp::Node{
public:
    KickBallNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~KickBallNode() override;

private:
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;
    void subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    std::shared_ptr<OrderInterpreter> order_interpreter_;
    bool no_target = true;
};



#endif //KICK_BALL_H_