#include "wheeled_bipedal_controllers/VMC.h"

void cal_VMC(double &T_1, double &T_2, kinematics::fwdKinematicsResult fKParams, double F, double T_p)
{
    double Tmat11 = L1 * sin(fKParams.phi0 - fKParams.phi3) * sin(fKParams.phi1 - fKParams.phi2) / sin(fKParams.phi3 - fKParams.phi2);
    double Tmat12 = L1 * cos(fKParams.phi0 - fKParams.phi3) * sin(fKParams.phi1 - fKParams.phi2) / fKParams.L0 / sin(fKParams.phi3 - fKParams.phi2);
    double Tmat21 = L4 * sin(fKParams.phi0 - fKParams.phi2) * sin(fKParams.phi3 - fKParams.phi4) / sin(fKParams.phi3 - fKParams.phi2);
    double Tmat22 = L4 * cos(fKParams.phi0 - fKParams.phi2) * sin(fKParams.phi3 - fKParams.phi4) / fKParams.L0 / sin(fKParams.phi3 - fKParams.phi2);

    T_1 = Tmat11 * F + Tmat12 * T_p;
    T_2 = Tmat21 * F + Tmat22 * T_p;
}

std::vector<double> cal_supportForce(double F, double T_p, double theta, double L_0, double m_w,
                                     double ddz_M, double dTheta, double ddTheta, double dL_0, double ddL_0)
{
    double P = F * cos(theta) + T_p * sin(theta) / L_0;

    double DDz_w = ddz_M - ddL_0 * cos(theta) + 2 * dL_0 * dTheta * sin(theta) + L_0 * ddTheta * sin(theta) + L_0 * dTheta * dTheta * cos(theta);

    std::vector<double> result;
    result.push_back(m_w * DDz_w + P + m_w * 9.81);
    result.push_back(DDz_w);
    return result;
    // return m_w * DDz_w + P + m_w * 9.81;
}