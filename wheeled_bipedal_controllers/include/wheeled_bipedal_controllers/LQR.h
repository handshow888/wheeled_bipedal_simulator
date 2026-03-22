#ifndef LQR_H
#define LQR_H

#include <vector>
#include <cmath>

// polynomial coefficients 多项式系数（低次在前）
extern std::vector<double> K11poly;
extern std::vector<double> K12poly;
extern std::vector<double> K13poly;
extern std::vector<double> K14poly;
extern std::vector<double> K15poly;
extern std::vector<double> K16poly;
extern std::vector<double> K21poly;
extern std::vector<double> K22poly;
extern std::vector<double> K23poly;
extern std::vector<double> K24poly;
extern std::vector<double> K25poly;
extern std::vector<double> K26poly;

extern double Kmat[12];

void calKmat(double legLength); // 根据腿长用多项式计算K矩阵插值

void cal_LQR_u(double theta, double thetaDot, double x, double xDot, double phi, double phiDot, double &T, double &T_p);

#endif