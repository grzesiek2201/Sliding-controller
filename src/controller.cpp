#include <math.h>
#include <cmath>

#include "controller.hpp"

int sat(double s)
{
    if (s > 1)
    {
        return 1;
    }
    else if (s < -1)
    {
        return -1;
    }
    else
    {
        return s;
    }
};

Controller::Controller(double k1, double k2)
: k1_(k1), k2_(k2)
{
    std::fill(prev_qr_.begin(), prev_qr_.end(), 0);
}

std::vector<double> Controller::scr(std::vector<double> pc, std::vector<double> pr, std::vector<double> qc, std::vector<double> qr)
{
    std::vector<double> pe = error_transform(pc, pr);
    double s1 = pe.at(0);
    double s2 = pe.at(2) + atan(qr.at(0) * pe.at(2));  //  POTENTIAL PROBLEM, WITH ATAN2; ARCTAN IN ORIGINAL CODE
    double ar = qr.at(0) - prev_qr_.at(0);
    double v = pe.at(1) * qc.at(1) + qr.at(0) * cos(pe.at(2)) + k1_ * sat(s1);
    double w = (pe.at(1) / (1 + pow((qr.at(0) * pe.at(1)), 2)) * ar) + 
                (qr.at(0) / pow((qr.at(0) * pe.at(1)), 2) * qr.at(0)) * sin(pe.at(2)) + k2_ * sat(s2);
    
    return std::vector<double> {v, w};
}

std::vector<double> Controller::error_transform(std::vector<double>& pc, std::vector<double>& pr)
{
    std::vector<double> ph(3);
    std::transform(pr.begin(), pr.end(), pc.begin(), ph.begin(), std::minus<double>());
    if (abs(ph.at(2)) > M_PI)
    {
        ph.at(2) = std::fmod(pr.at(2), (2*M_PI)) - std::fmod(pc.at(2), (2*M_PI));
    }
    double tc = pc.at(2);
    std::vector<double> pe(3);
    pe.at(0) = cos(tc) * ph.at(0) + sin(tc) * ph.at(1);
    pe.at(1) = -sin(tc) * ph.at(0) + cos(tc) * ph.at(1);
    pe.at(2) = ph.at(2); 

    return pe;
}