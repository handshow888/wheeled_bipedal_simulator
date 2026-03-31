#ifndef VMC_H
#define VMC_H

#include <cmath>
#include "structural_params.h"
#include "kinematics.h"

void cal_VMC(double &T_1, double &T_2, kinematics::fwdKinematicsResult fKParams, double F, double T_p);

#endif