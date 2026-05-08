#pragma once

#include <Eigen/Dense>

/*

theta = [a,b]
y_hat = (Phi_k)^T * (theta_k-1)_hat
ek = yk-yk_hat

y_hat = a*x + b
算误差：e = y - y_hat
算这次该改多少：K = .../denom
更新参数：(a,b) += K*e
更新内部记忆 P
*/


struct RLS_Params
{
    double Lambda = 1.0;
    double Confidence = 1e6;
    int ThresholdPoint = 10;
};




class RLSLine
{
public:

    RLSLine(double Lambda, double Confidence, int ThresholdPoint);
    RLSLine(RLS_Params params);
    ~RLSLine()=default;

    
    void UpdateLineParams(double x, double y);  // 迭代
    void Reset();                               // 重置
    bool GetLineParams(double& a, double& b);



private:

    double Lambda = 1.0;
    double Confidence = 1e6;
    int ThresholdPoint = 0;
    int PointCnt;
    double Now_a;
    double Now_b;

    Eigen::Vector2d Theta_;
    Eigen::Matrix2d P_;
    

};


