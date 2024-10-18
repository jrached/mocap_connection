#include <memory>

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
//#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MocapConnection : public rclcpp::Node
{
    public: 
    
    MocapConnection() : Node("mocap_connection")
    {
        // Define quality of service profile
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile); 

        // Define publishers and subscriptions
        mocap_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/PX01/world", qos, std::bind(&MocapConnection::mocap_callback, this, _1));
        px4_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos); 
        odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&MocapConnection::odom_callback, this, _1));

        //offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	
    	// Make 40 Hz timer to publish mocap data
        timer_ = this->create_wall_timer(25ms, std::bind(&MocapConnection::timer_callback, this));
    }

    private:

    // Declare publishers and subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_publisher_;
    //rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscription_;

    // Declare timer and pose message and odom msg
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped pose_msg;
    px4_msgs::msg::VehicleOdometry odom_msg;

    void odom_callback(const px4_msgs::msg::VehicleOdometry& msg){
        // Store odometry message in class variable to sync with its timestamp and timestamp_sample
        odom_msg = msg;
    }

    
    void mocap_callback(const geometry_msgs::msg::PoseStamped& msg){
        // Store mocap data in class variable to be published at different rate
        pose_msg = msg;
    }

    void timer_callback(){
        // Publish mocap data under new topic at 40 Hz
        auto vis_odom_msg = px4_msgs::msg::VehicleOdometry();
        
	    // vis_odom_msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
        vis_odom_msg.timestamp = odom_msg.timestamp;
        vis_odom_msg.timestamp_sample = odom_msg.timestamp_sample;
        vis_odom_msg.position[0] =pose_msg.pose.position.y;
        vis_odom_msg.position[1] =pose_msg.pose.position.x;
        vis_odom_msg.position[2] = -pose_msg.pose.position.z;
        vis_odom_msg.q[0] =pose_msg.pose.orientation.w;
        vis_odom_msg.q[1] =pose_msg.pose.orientation.y;
        vis_odom_msg.q[2] =pose_msg.pose.orientation.x;
        vis_odom_msg.q[3] = -pose_msg.pose.orientation.z;

        this->px4_publisher_->publish(vis_odom_msg);

	// Publish heartbeat
	//px4_msgs::msg::OffboardControlMode msg{};
	//msg.position = true;
	//msg.velocity = false;
	//msg.acceleration = false;
	//msg.attitude = false;
	//msg.body_rate = false;
	//msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	//offboard_control_mode_publisher_->publish(msg);

    }




};

int main(int argc, char* argv[])
{
    std::cout << "Starting mocap connection node... " << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapConnection>());
    rclcpp::shutdown();
    return 0;
}
