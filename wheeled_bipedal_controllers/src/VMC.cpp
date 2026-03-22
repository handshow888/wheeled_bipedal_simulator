#include "wheeled_bipedal_controllers/VMC.h"

void cal_VMC(double &T_1, double &T_2, kinematics::fwdKinematicsResult fwdParams, double F, double T_p)
{
    double Tmat11 = L1 * sin(fwdParams.phi0 - fwdParams.phi3) * sin(fwdParams.phi1 - fwdParams.phi2) / sin(fwdParams.phi3 - fwdParams.phi2);
    double Tmat12 = L1 * cos(fwdParams.phi0 - fwdParams.phi3) * sin(fwdParams.phi1 - fwdParams.phi2) / fwdParams.L0 / sin(fwdParams.phi3 - fwdParams.phi2);
    double Tmat21 = L4 * sin(fwdParams.phi0 - fwdParams.phi2) * sin(fwdParams.phi3 - fwdParams.phi4) / sin(fwdParams.phi3 - fwdParams.phi2);
    double Tmat22 = L4 * cos(fwdParams.phi0 - fwdParams.phi3) * sin(fwdParams.phi3 - fwdParams.phi4) / fwdParams.L0 / sin(fwdParams.phi3 - fwdParams.phi2);

    T_1 = Tmat11 * F + Tmat12 * T_p;
    T_2 = Tmat21 * F + Tmat22 * T_p;
}