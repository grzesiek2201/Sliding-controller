class Controller
{
    public:
        Controller(double k1=1, double k2=1);
        std::vector<double> scr(std::vector<double> pc, std::vector<double> pr, std::vector<double> qc, std::vector<double> qr);
        std::vector<double> error_transform(std::vector<double>& pc, std::vector<double>& pr);
    private:
        double k1_;
        double k2_;
        std::vector<double> prev_qr_ = {.0, .0, .0};
};