#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"

class MessageToTF : public rclcpp::Node
{
public:
    MessageToTF()
        : Node("message_to_tf")
    {
        broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        this->declare_parameter<std::string>(
            "parent_frame_id",
            "world");

        this->declare_parameter<std::string>(
            "child_frame_id",
            "base_link");

        parent_frame_id_ =
            this->get_parameter("parent_frame_id").as_string();

        child_frame_id_ =
            this->get_parameter("child_frame_id").as_string();

        sub_ = this->create_subscription<
            geometry_msgs::msg::PoseStamped>(
            "pose",
            10,
            std::bind(
                &MessageToTF::poseCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "message_to_tf node started");
    }

private:
    void poseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = parent_frame_id_;
        tf_msg.child_frame_id = child_frame_id_;

        tf_msg.transform.translation.x =
            msg->pose.position.x;

        tf_msg.transform.translation.y =
            msg->pose.position.y;

        tf_msg.transform.translation.z =
            msg->pose.position.z;

        tf_msg.transform.rotation =
            msg->pose.orientation;

        broadcaster_->sendTransform(tf_msg);
    }

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    std::shared_ptr<
        tf2_ros::TransformBroadcaster> broadcaster_;

    std::string parent_frame_id_;
    std::string child_frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MessageToTF>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
