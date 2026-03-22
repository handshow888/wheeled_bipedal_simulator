#include "wheeled_bipedal_controllers/LQR.h"

// polynomial coefficients 多项式系数（低次在前）
std::vector<double> K11poly;
std::vector<double> K12poly;
std::vector<double> K13poly;
std::vector<double> K14poly;
std::vector<double> K15poly;
std::vector<double> K16poly;
std::vector<double> K21poly;
std::vector<double> K22poly;
std::vector<double> K23poly;
std::vector<double> K24poly;
std::vector<double> K25poly;
std::vector<double> K26poly;

double Kmat[12] = {0.0};

void cal_LQR_u(double theta, double thetaDot, double x, double xDot, double phi, double phiDot, double &T, double &T_p)
{
    T = -Kmat[0] * theta;
    T -= Kmat[1] * thetaDot;
    T -= Kmat[2] * x;
    T -= Kmat[3] * xDot;
    T -= Kmat[4] * phi;
    T -= Kmat[5] * phiDot;

    T_p = -Kmat[6] * theta;
    T_p -= Kmat[7] * thetaDot;
    T_p -= Kmat[8] * x;
    T_p -= Kmat[9] * xDot;
    T_p -= Kmat[10] * phi;
    T_p -= Kmat[11] * phiDot;
}

void calKmat(double legLength)
{
    for (int i = 0; i < 12; ++i)
    {
        Kmat[i] = 0;
    }
    int polySize = (int)K11poly.size();
    for (int i = 0; i < polySize; ++i)
    {
        double pow_legLength_i = pow(legLength, i);
        Kmat[0] += K11poly[i] * pow_legLength_i;
        Kmat[1] += K12poly[i] * pow_legLength_i;
        Kmat[2] += K13poly[i] * pow_legLength_i;
        Kmat[3] += K14poly[i] * pow_legLength_i;
        Kmat[4] += K15poly[i] * pow_legLength_i;
        Kmat[5] += K16poly[i] * pow_legLength_i;
        Kmat[6] += K21poly[i] * pow_legLength_i;
        Kmat[7] += K22poly[i] * pow_legLength_i;
        Kmat[8] += K23poly[i] * pow_legLength_i;
        Kmat[9] += K24poly[i] * pow_legLength_i;
        Kmat[10] += K25poly[i] * pow_legLength_i;
        Kmat[11] += K26poly[i] * pow_legLength_i;
    }
}