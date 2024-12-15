#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "cmath"
#include "Eigen/Dense"

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

    Eigen::Matrix3d rotation_x(double roll) {
    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, std::cos(roll), -std::sin(roll),
         0, std::sin(roll), std::cos(roll);
    return R;
    }

    Eigen::Matrix3d rotation_y(double pitch) {
    Eigen::Matrix3d R;
    R << std::cos(pitch), 0, std::sin(pitch),
         0, 1, 0,
         -std::sin(pitch), 0, std::cos(pitch);
    return R;
    }

    Eigen::Matrix3d rotation_z(double yaw) {
    Eigen::Matrix3d R;
    R << std::cos(yaw), -std::sin(yaw), 0,
         std::sin(yaw), std::cos(yaw), 0,
         0, 0, 1;
    return R;
    }


    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        get_tf(vector, "map", "base_footprint");
        
        xpub=vector[1] - 3;
        ypub=-vector[0] + 4.5;
        
        get_tf(aruco_cam, "map", "aruco_marker_frame");
        
        double x_ar = aruco_cam[1] - 3;
        double y_ar = -aruco_cam[0] + 4.5;
        double z_ar = aruco_cam[2] + 0.1;
        
        double qx = aruco_cam[3];
        double qy = aruco_cam[4];
        double qz = aruco_cam[5];
        double qw = aruco_cam[6];
        
        
        std::array<double, 4> q = {qx, qy, qz, qw};
        double yaw = -M_PI / 2;
        /*std::array<double, 4> q_yaw = {0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)};
        
        double qx_ar = q_yaw[3] * q[0] + q_yaw[0] * q[3] + q_yaw[1] * q[2] - q_yaw[2] * q[1];
        double qy_ar = q_yaw[3] * q[1] - q_yaw[0] * q[2] + q_yaw[1] * q[3] + q_yaw[2] * q[0];
        double qz_ar = q_yaw[3] * q[2] + q_yaw[0] * q[1] - q_yaw[1] * q[0] + q_yaw[2] * q[3];
        double qw_ar = q_yaw[3] * q[3] - q_yaw[0] * q[0] - q_yaw[1] * q[1] - q_yaw[2] * q[2];
        
        std::cout<<"aruco: " <<"x: " <<x_ar <<" y: " <<y_ar <<" z: " <<z_ar <<" q: [" <<qx_ar <<", " <<qy_ar <<", " <<qz_ar <<", " <<qw_ar<<"]\n";
        */
        double r = std::atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));
        double p = std::asin(2*(qw*qy-qz*qx));
        double y = std::atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
        
        //std::cout<<"RPY: " <<r <<"    " <<p <<"    " <<y <<"\n";
        
        Eigen::Matrix3d R_fixed_to_world = rotation_z(yaw);
        Eigen::Matrix3d R_point_in_fixed_frame = rotation_z(y) * rotation_y(p) * rotation_x(r);
        Eigen::Matrix3d R_total = R_fixed_to_world * R_point_in_fixed_frame;

        double roll = atan2(R_total(2, 1), R_total(2, 2));
        double pitch = -asin(R_total(2, 0));
        double yaww = atan2(R_total(1, 0), R_total(0, 0));
        
        std::cout<<"RPY: " <<roll <<"    " <<pitch <<"    " <<yaww <<"\n";
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
    std::array<double, 7> aruco_cam;
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

