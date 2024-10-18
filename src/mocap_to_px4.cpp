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
        mocap_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/PX01/world", 10, std::bind(&MocapConnection::mocap_callback, this, _1));
        px4_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10); 

        //offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	
	// Make timer for 40 Hz
        timer_ = this->create_wall_timer(25ms, std::bind(&MocapConnection::timer_callback, this));
    }

    private:

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_publisher_;
    //rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

    // Declare timer and pose message
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped pose_msg;
    
    void mocap_callback(const geometry_msgs::msg::PoseStamped& msg){
        // Store mocap data in class variable to be published at different rate
        pose_msg = msg;
    }

    void timer_callback(){
        // Publish mocap data under new topic at 40 Hz
        auto odom_msg = px4_msgs::msg::VehicleOdometry();
        
	odom_msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
        odom_msg.position[0] =pose_msg.pose.position.y;
        odom_msg.position[1] =pose_msg.pose.position.x;
        odom_msg.position[2] = -pose_msg.pose.position.z;
        odom_msg.q[0] =pose_msg.pose.orientation.w;
        odom_msg.q[1] =pose_msg.pose.orientation.y;
        odom_msg.q[2] =pose_msg.pose.orientation.x;
        odom_msg.q[3] = -pose_msg.pose.orientation.z;

        this->px4_publisher_->publish(odom_msg);

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
