#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "follower.cpp"
#include "controller.cpp"

#include "json.hpp"
#include <fstream>
#include <iostream>

using namespace std::chrono;


std::vector<double> euler_from_quaternion(geometry_msgs::msg::Quaternion& quaternion)
{
    /*
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    */
    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    double w = quaternion.w;

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    double roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    double pitch = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    return std::vector<double> {roll, pitch, yaw};
}


class Rider : public rclcpp::Node
{
    public:
        Rider()
        : Node("rider_node")
        {
            start_ = false;

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot0/cmd_vel", 1);
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>("robot0/ground_truth_pos", 1, std::bind(&Rider::pose_callback, this, std::placeholders::_1));
            start_sub_ = this->create_subscription<std_msgs::msg::Bool>("robot/start", 1, std::bind(&Rider::start_movement, this, std::placeholders::_1));

            data = load_data("/home/grzesiek/dev_ws/src/coop_cpp/resources/trajectory.json");

            controller_ = std::make_unique<Controller>(.2, .2);

            timer_ = this->create_wall_timer(10ms, std::bind(&Rider::update_vel, this));

            ts_ = data["T"].front(); data["T"].pop();
            data["T"].pop();

            follower_ = std::make_unique<Follower>(.35, M_PI/2, std::vector<double>{data["X"].front(), data["Y"].front(), data["Theta"].front()});
            x_prev_ = std::vector<double>{data["X"].front(), data["Y"].front(), data["Theta"].front()};

            std::string x_prevv = "x:" + std::to_string(x_prev_.at(0)) + " y:" + std::to_string(x_prev_.at(1)) + "theta:" + std::to_string(x_prev_.at(2));
            RCLCPP_INFO(this->get_logger(), x_prevv.c_str());

            std::vector<double> x0_fol = follower_->get_state();
            std::string x0_fol_str = "x:" + std::to_string(x0_fol.at(0)) + " y:" + std::to_string(x0_fol.at(1)) + "theta:" + std::to_string(x0_fol.at(2));
            RCLCPP_INFO(this->get_logger(), x0_fol_str.c_str());
        }

    private:
        std::map<std::string, std::queue<double>> load_data(std::string filename)
        {
            std::ifstream ifs(filename);
            auto j = nlohmann::ordered_json::parse(ifs);
            // ifs >> j;
            
            std::map<std::string, std::queue<double>> loaded_values;
            for ( auto const& el: j["x"]) {
                loaded_values["X"].push(el);
            }
            for ( auto const& el: j["y"]) {
                loaded_values["Y"].push(el);
            }
            for ( auto const& el: j["0"]) {
                loaded_values["Theta"].push(el);
            }
            for ( auto const& el: j["v"]) {
                loaded_values["V"].push(el);
            }
            for ( auto const& el: j["w"]) {
                loaded_values["W"].push(el);
            }
            for ( auto const& el: j["t"]) {
                loaded_values["T"].push(el);
            }

            return loaded_values;
        }
        
        void start_movement(std_msgs::msg::Bool::SharedPtr msg)
        {
            start_ = true;
            t0 = steady_clock::now();
            t1 = steady_clock::now();
            start_sub_.reset();  // unsubscribe when going out of scope
            update_vel();
        }

        void update_vel()
        {
            if ( !start_ )
            {
                return;
            }

            if ( data["T"].empty() )
            {
                send_vel(std::vector<double> {.0, .0});
                return;
            }

            t1 = steady_clock::now();
            std::vector<double> u{0, 0};

            if ( std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() >= ts_*1000 )
            {
                // virtual leader state at the end of this time interval
                double x = data["X"].front(); data["X"].pop();
                double y = data["Y"].front(); data["Y"].pop();
                double theta = data["Theta"].front(); data["Theta"].pop();
                // virtual leader control at this time interval

                u.at(0) = data["V"].front(); data["V"].pop();
                u.at(1) = data["W"].front(); data["W"].pop();

                // next time interval
                double t = data["T"].front(); data["T"].pop();

                // update reference position and velocity of the follower
                std::vector<double> state{x, y, theta};

                follower_->update_pos(state, x_prev_, u);
                follower_->update_vel(t - ts_);
                std::string vel_fol = "v:" + std::to_string(follower_->get_control().at(0)) + "; w:" + std::to_string(follower_->get_control().at(1));
                RCLCPP_INFO(this->get_logger(), vel_fol.c_str());

                // calculate control values
                std::vector<double> controls(2);
                controls = controller_->scr(follower_->get_state(), follower_->get_state_ref(), follower_->get_control(), u);
                send_vel(controls);
                // printing to verify correctness
                std::vector<double> follower_state = follower_->get_state();
                std::vector<double> follower_ref_state = follower_->get_state_ref();
                std::vector<double> error = controller_->error_transform(follower_state, follower_ref_state);
                std::string error_str = "x: " + std::to_string(error.at(0)) + "y: " + std::to_string(error.at(1)) + "theta: " + std::to_string(error.at(2));
                RCLCPP_INFO(this->get_logger(), error_str.c_str());

                ts_ = t;
                RCLCPP_INFO(this->get_logger(), std::to_string(t).c_str());
                x_prev_ = std::vector<double>{x, y, theta};
            }
        }

        void send_vel(std::vector<double> u)
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = u[0];
            msg.angular.z = u[1];
            pub_->publish(msg);
        }

        void pose_callback(nav_msgs::msg::Odometry::SharedPtr msg)
        {
            double x = double(msg->pose.pose.position.x);
            double y = double(msg->pose.pose.position.y);
            std::vector<double> angles = euler_from_quaternion(msg->pose.pose.orientation);
            std::vector<double> state{x, y, angles.at(2)};
            follower_->set_state(state);
        }
        
        void timer_callback(std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "TIMER CALLBACK");
        }

        bool start_;
        steady_clock::time_point t0;
        steady_clock::time_point t1;
        double ts_;
        std::map<std::string, std::queue<double>> data;
        std::vector<double> x_prev_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

        std::unique_ptr<Controller> controller_;// = std::make_unique<Controller>();
        std::unique_ptr<Follower> follower_;// = std::make_unique<Follower>();

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rider>());
    rclcpp::shutdown();
    return 0;
}