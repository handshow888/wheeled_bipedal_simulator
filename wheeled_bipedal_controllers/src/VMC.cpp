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