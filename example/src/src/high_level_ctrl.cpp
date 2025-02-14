// Goal: to make the go2 move to a point (pose) as a service call.
// front, back, left, right.

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

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
    };

private:
    void timer_callback()
    {
        t += dt;
        if (t > 0)
        {
            double time_seg = 0.2;
            double time_temp = t - time_seg;

            std::vector<PathPoint> path;

            // // Give a forward path motion
            // float vx = 0.1;
            // float vy = 0.0;
            // float vyaw = 0.0;
            // // Give a forward path.
            // sport_req.Move(req, vx, vy, vyaw);

            // // Give a backward motion : diagonally baackgrack
            // float vx = - 0.1;
            // float vy = 0.0;
            // float vyaw = 0.0;
            // // Give a forward path.
            // sport_req.Move(req, vx, vy, vyaw);

            // // Give a left motion
            // float vx = 0.0;
            // float vy = +0.1;
            // float vyaw = 0.0;
            // // Give a forward path.
            // sport_req.Move(req, vx, vy, vyaw);

            // // Give a right motion
            // float vx = 0.0;
            // float vy = -0.1;
            // float vyaw = 0.0;
            // // Give a forward path.
            // sport_req.Move(req, vx, vy, vyaw);


            // Try damping req first. : works
            // sport_req.Damp(req);


            // Not tested.
            // // Turn left.
            // float vx = 0.0;
            // float vy = 0.0;
            // float vyaw = 0.1;
            // // Give a forward path.
            // sport_req.Move(req, vx, vy, vyaw);

            // Turn right.
            float vx = 0.0;
            float vy = 0.0;
            float vyaw = -0.1;
            // Give a forward path.
            sport_req.Move(req, vx, vy, vyaw);


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

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<high_level_ctrl>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
