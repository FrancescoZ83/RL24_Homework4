#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class Fra2moPoseXYNode : public rclcpp::Node
{
public:
    Fra2moPoseXYNode() : Node("fra2mo_pose_xy_node")
    {
        this->declare_parameter<std::string>("tf_topic", "/model/fra2mo/tf");
        this->declare_parameter<std::string>("xy_topic", "/xy_bag");

        auto tf_topic = this->get_parameter("tf_topic").as_string();
        auto xy_topic = this->get_parameter("xy_topic").as_string();

        tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic,
            10,
            std::bind(&Fra2moPoseXYNode::tfCallback, this, std::placeholders::_1));

        xy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            xy_topic,
            10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Fra2moPoseXYNode::publishXY, this));

        RCLCPP_INFO(this->get_logger(), "fra2mo_pose_xy_node initialized.");
    }

private:

    void get_tf(std::array<double, 7>& output, const std::string& tf_base, const std::string& tf_end) {
    try {
        geometry_msgs::msg::TransformStamped temp_pose;
        temp_pose = tf_buffer_->lookupTransform(tf_base, tf_end, rclcpp::Time(0), std::chrono::milliseconds(100));
        tf2::Transform tf3d;
        tf2::fromMsg(temp_pose.transform, tf3d);
        tf2::Vector3 translation = tf3d.getOrigin();
        tf2::Quaternion rotation = tf3d.getRotation();
        
        output[0] = translation.x();
        output[1] = translation.y();
        output[2] = translation.z();
        output[3] = rotation.x();
        output[4] = rotation.y();
        output[5] = rotation.z();
        output[6] = rotation.w();
    } catch (const tf2::TransformException&) {}
}



    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        get_tf(vector, "map", "base_footprint");
        
        xpub=vector[1] - 3;
        ypub=-vector[0] + 4.5;
        
    }

    void publishXY()
    {
        std_msgs::msg::Float64MultiArray xy_msg;
        xy_msg.data = {xpub, ypub};
        xy_publisher_->publish(xy_msg);
        RCLCPP_INFO(this->get_logger(), "Published x: %.2f, y: %.2f", xpub, ypub);
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr xy_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::array<double, 7> vector = {0,0,0,0,0,0,0};
    double xpub;
    double ypub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fra2moPoseXYNode>());
    rclcpp::shutdown();
    return 0;
}
