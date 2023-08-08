#include <vector>
#include <algorithm>

#include "follower.hpp"

#include "rclcpp/rclcpp.hpp"


Follower::Follower(double dist, double alpha, std::vector<double> x0)
: dist_(dist), alpha_(alpha)
{
    double x_transf = dist_ * cos(x0.at(2) + alpha_);
    double y_transf = dist_ * sin(x0.at(2) + alpha_);
    std::vector<double> init_transf {x_transf, y_transf, 0};

    state_ = x0;
    state_ref_ = x0;
    state_ref_prev_ = x0;

    std::transform(state_.begin(), state_.end(), init_transf.begin(), state_.begin(), std::plus<double>());
    std::transform(state_ref_.begin(), state_ref_.end(), init_transf.begin(), state_ref_.begin(), std::plus<double>());
    std::transform(state_ref_prev_.begin(), state_ref_prev_.end(), init_transf.begin(), state_ref_prev_.begin(), std::plus<double>());
}

void Follower::update_vel(double ts)
{
    double vx = (state_ref_.at(0) - state_ref_prev_.at(0)) / ts;
    double vy = (state_ref_.at(1) - state_ref_prev_.at(1)) / ts;
    double v = sqrt(vx*vx + vy*vy);
    double w = (state_ref_.at(2) - state_ref_prev_.at(2)) / ts;
    control_.at(0) = v;
    control_.at(1) = w;
}

void Follower::update_pos(std::vector<double>& x_vl, std::vector<double>& x_vl_prev, std::vector<double>& u_vl)
{
    double x_new = x_vl.at(0) + dist_ * cos(alpha_ + x_vl_prev.at(2));
    double y_new = x_vl.at(1) + dist_ * sin(alpha_ + x_vl_prev.at(2));
    double theta_new = x_vl_prev.at(2);
    if (round(u_vl.at(1) * pow(10, 16)) / pow(10, 16) != 0)
    {
        theta_new = atan2((y_new - state_ref_.at(1)), (x_new - state_ref_.at(0)));
    }
    // state_ref_prev_.at(0) = 1;
    // state_ref_.at(0) = 0;
    // state_ref_prev_.at(0) = state_ref_.at(0); state_ref_prev_.at(1) = state_ref_.at(1); state_ref_prev_.at(2) = state_ref_.at(2);
    state_ref_prev_ = state_ref_;
    state_ref_.at(0) = x_new; state_ref_.at(1) = y_new; state_ref_.at(2) = theta_new;
}

void Follower::set_state(std::vector<double>& state)
{
    state_ = state;
}

std::vector<double> Follower::get_state()
{
    return state_;
}

std::vector<double> Follower::get_state_ref()
{
    return state_ref_;
}

std::vector<double> Follower::get_control()
{
    return control_;
}