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
#include "tf2_ros/static_transform_broadcaster.h"

class Fra2moPoseXYNode : public rclcpp::Node
{
public:
    Fra2moPoseXYNode() : Node("fra2mo_pose_xy_node")
    {
        // topics
        this->declare_parameter<std::string>("tf_topic", "/model/fra2mo/tf");
        this->declare_parameter<std::string>("xy_topic", "/xy_bag");
        auto tf_topic = this->get_parameter("tf_topic").as_string();
        auto xy_topic = this->get_parameter("xy_topic").as_string();

        // fra2mo pose subscriber
        tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic,
            10,
            std::bind(&Fra2moPoseXYNode::tfCallback, this, std::placeholders::_1));

        // fra2mo (x,y) position publisher (needed for bag recording)
        xy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            xy_topic,
            10);

        // timer for (x,y) publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Fra2moPoseXYNode::publishXY, this));
        
        // tf_static publisher for the aruco pose
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
       
        RCLCPP_INFO(this->get_logger(), "fra2mo_pose_xy_node initialized.");
    }

private:

    // Gets the tf between two frames and saves it in a standard vector
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
         }catch (const tf2::TransformException&) {}
    }
    
    // Computes the quaternion multipilication
    std::array<double, 4> quaternionMultiply(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
        double x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
        double x2 = q2[0], y2 = q2[1], z2 = q2[2], w2 = q2[3];

        return {
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        };
    }


    // subscriber: getting (x,y) position of fra2mo and retrieving the aruco pose
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        // retrieving the (x,y) position of fra2mo in the world frame
        get_tf(vector, "map", "base_footprint");
        xpub=vector[1] - 3;
        ypub=-vector[0] + 4.5;
        
        // getting the transform between the aruco frame and the map frame(this is the one centered in the robot initial position)
        get_tf(aruco_cam, "map", "aruco_marker_frame");
        
        // position adjustments
        double x_ar = aruco_cam[1] - 3;
        double y_ar = -aruco_cam[0] + 4.5;
        double z_ar = aruco_cam[2] + 0.1;
        
        // orientation adjustments
        double qx = aruco_cam[3];
        double qy = aruco_cam[4];
        double qz = aruco_cam[5];
        double qw = aruco_cam[6];
        std::array<double, 4> q = {qx, qy, qz, qw};
        std::array<double, 4> q_rel = {0, 0, -0.707, 0.707}; //Adjustment Rotation of yaw = -pi/2
        std::array<double, 4> q_result = quaternionMultiply(q_rel, q);
        qx=q_result[0];
        qy=q_result[1];
        qz=q_result[2];
        qw=q_result[3];
        
        // final aruco pose
        tf_vector_pub = {x_ar,y_ar,z_ar,qx,qy,qz,qw};
    }

    // publisher
    void publishXY()
    {   // (x,y) fra2mo position publishing
        std_msgs::msg::Float64MultiArray xy_msg;
        xy_msg.data = {xpub, ypub};
        xy_publisher_->publish(xy_msg);
        RCLCPP_INFO(this->get_logger(), "fra2mo position x: %.2f, y: %.2f", xpub, ypub);
        
        // tf_static publisher
        if (tf_vector_pub[6] != 0 && flag==0){ // if tf_vector_pub[6] = qw is 0 (which is the starting value) it means that the pose hasn't been already detected
            double dist=std::sqrt((tf_vector_pub[0]-xpub)*(tf_vector_pub[0]-xpub)+(tf_vector_pub[1]-ypub)*(tf_vector_pub[1]-ypub)); // it ensures that it publishes only when it's near enough
            if(dist<0.75){
                this->make_transforms(tf_vector_pub); // publishing
                flag = 1; // it does it just once
            }
        }
    }
    
    // function that tf_static publishes the input std vector
    void make_transforms(const std::array<double, 7> vec)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "aruco_marker_frame";

        t.transform.translation.x = vec[0];
        t.transform.translation.y = vec[1];
        t.transform.translation.z = vec[2];
        t.transform.rotation.x = vec[3];
        t.transform.rotation.y = vec[4];
        t.transform.rotation.z = vec[5];
        t.transform.rotation.w = vec[6];

        tf_static_broadcaster_->sendTransform(t);
    }


    // internal parameters
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr xy_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    bool flag=0;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::array<double, 7> vector = {0,0,0,0,0,0,0};
    std::array<double, 7> aruco_cam;
    std::array<double, 7> tf_vector_pub = {0,0,0,0,0,0,0};
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

