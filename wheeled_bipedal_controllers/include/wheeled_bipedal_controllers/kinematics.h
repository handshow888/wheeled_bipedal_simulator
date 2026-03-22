
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "structural_params.h"

namespace kinematics
{
    typedef struct
    {
        float x, y;
    } point;

    typedef struct
    {
        point wheelPos;
        double L0;   // 两关节电机中点到轮子中心的距离
        double phi0; // 两关节电机中点到轮子中心点的角度(rad)
        double phi1, phi2, phi3, phi4;
    } fwdKinematicsResult;

    fwdKinematicsResult forwardKinematics(float phi1, float phi4);

} // namespace kinematics

#endif