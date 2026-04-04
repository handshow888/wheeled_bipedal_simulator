#ifndef VMC_H
#define VMC_H

#include <cmath>
#include <vector>
#include "structural_params.h"
#include "kinematics.h"

void cal_VMC(double &T_1, double &T_2, kinematics::fwdKinematicsResult fKParams, double F, double T_p);

std::vector<double> cal_supportForce(double F, double T_p, double theta, double L_0, double m_w,
                        double ddz_M, double dTheta, double ddTheta, double dL_0, double ddL_0);

#endif