#include "wheeled_bipedal_controllers/LQR.h"
#include "iostream"

namespace LQR
{
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
        double resultT = 0.0;
        double resultT_p = 0.0;
        
        resultT = -Kmat[0] * theta;
        resultT -= Kmat[1] * thetaDot;
        resultT -= Kmat[2] * x;
        resultT -= Kmat[3] * xDot;
        resultT -= Kmat[4] * phi;
        resultT -= Kmat[5] * phiDot;

        resultT_p = -Kmat[6] * theta;
        resultT_p -= Kmat[7] * thetaDot;
        resultT_p -= Kmat[8] * x;
        resultT_p -= Kmat[9] * xDot;
        resultT_p -= Kmat[10] * phi;
        resultT_p -= Kmat[11] * phiDot;

        T = resultT;
        T_p = resultT_p;
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
        // printf("Kmat:\n%.2f %.2f %.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f %.2f %.2f\n",
        //         Kmat[0],Kmat[1],Kmat[2],Kmat[3],Kmat[4],Kmat[5],
        //         Kmat[6],Kmat[7],Kmat[8],Kmat[9],Kmat[10],Kmat[11]);
    }
} // namespace LQR