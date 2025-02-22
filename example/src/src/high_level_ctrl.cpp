// Goal: to make the go2 move to a point (pose) as a service call.
// front, back, left, right.

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "unitree_go2_nav_interfaces/msg/nav_to_pose.hpp"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;


class high_level_ctrl : public rclcpp::Node
{
public:
    high_level_ctrl() : Node("high_level_ctrl")
    {
        // the state_suber is set to subscribe "high_level_ctrl" topic
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&high_level_ctrl::state_callback, this, _1));
        
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&high_level_ctrl::timer_callback, this));

        t = -1; // Runing time count

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.keep_last(50);  // Increase history depth
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // Or TRANSIENT_LOCAL if needed

        // Add a navigation subscriber, that publishes the sport request after processing stuff.
        nav_msg_sub = create_subscription<unitree_go2_nav_interfaces::msg::NavToPose>("nav_twist", qos_profile, std::bind(&high_level_ctrl::nav_msg_callback, this, _1));
    };

private:
    void timer_callback()
    {
        t += dt;
        if (t > 0)
        {
            std::vector<PathPoint> path;

            // Try damping req first. : works
            // sport_req.Damp(req);

            // Nav to pose
            float vx = cmd_vel.linear.x;
            float vy = cmd_vel.linear.y;
            float vyaw = cmd_vel.angular.z;
            // Give a forward path.
            sport_req.Move(req, vx, vy, vyaw);


            // When it reached
            float tollerance = 0.001;
            if (std::abs(vx) < tollerance && std::abs(vy) < tollerance && std::abs(vyaw) < tollerance)
            {
                // Stop the robot
                sport_req.StopMove(req);
                RCLCPP_INFO(get_logger(), "[HIGH LEVEL CONTROLLER] /////////////////////////////////////////////////////////////////////////////////.");
                RCLCPP_INFO(get_logger(), "[HIGH LEVEL CONTROLLER] Goal Reached, Robot stopped.");
            }

            // Publish request messages
            req_puber->publish(req);
        }
    };

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current position of robot when t<0
        // This position is used as the initial coordinate system
        // Basically odom.

        if (t < 0)
        {
            // Get initial position
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            std::cout << "[Odom]";
            std::cout << px0 << ", " << py0 << ", " << yaw0 << std::endl;
        }
    }

    // Member variables.
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Subscription<unitree_go2_nav_interfaces::msg::NavToPose>::SharedPtr nav_msg_sub;
    geometry_msgs::msg::Twist cmd_vel;

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle


    void nav_msg_callback(const unitree_go2_nav_interfaces::msg::NavToPose msg)
    {
        cmd_vel.linear.x = msg.x;
        cmd_vel.linear.y = msg.y;
        cmd_vel.angular.z = msg.yaw;
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<high_level_ctrl>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
