#include <vector>


class Follower
{
    public:
        Follower(double dist=0, double alpha=0, std::vector<double> x0=std::vector<double>{0, 0, 0});
        void update_vel(double ts);
        void update_pos(std::vector<double>& x_vl, std::vector<double>& x_vl_prev, std::vector<double>& u_vl);
        void set_state(std::vector<double>& state);
        std::vector<double> get_state();
        std::vector<double> get_state_ref();
        std::vector<double> get_control();
    private:
        double dist_;
        double alpha_;
        std::vector<double> state_{.0, .0, .0};
        std::vector<double> state_ref_{.0, .0, .0};
        std::vector<double> state_ref_prev_{.0, .0, .0};
        std::vector<double> control_{.0, .0};

};