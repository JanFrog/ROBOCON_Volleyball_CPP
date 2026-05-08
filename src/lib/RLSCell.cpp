#include"RLSCell.h"



RLSLine::RLSLine(double Lambda,double Confidence,int ThresholdPoint):
    Lambda(Lambda),
    Confidence(Confidence),
    ThresholdPoint(ThresholdPoint),
    PointCnt(0),
    Now_a(0.0),
    Now_b(0.0),
    Theta_(0.0, 0.0),
    P_(Eigen::Matrix2d::Identity() * Confidence) {}





RLSLine::RLSLine(RLS_Params params):
    RLSLine(params.Lambda,params.Confidence,params.ThresholdPoint){};




void RLSLine::UpdateLineParams(double x, double y)
{
    this->PointCnt++;

    const Eigen::Vector2d phi(x, 1.0);

    const double y_hat = Theta_(0) * x + Theta_(1);
    const double e = y - y_hat;

    const double denom = this->Lambda + (phi.transpose() * P_ * phi)(0, 0);
    const Eigen::Vector2d K = (P_ * phi) / denom;

    this->Theta_ += K * e;
    this->P_ = (this->P_ - K * (phi.transpose() * P_)) / this->Lambda;

    this->Now_a = this->Theta_(0);
    this->Now_b = this->Theta_(1);

}



void RLSLine::Reset()
{
    this->PointCnt = 0;
    this->Now_a = 0;
    this->Now_b = 0;
    this->Theta_ = Eigen::Vector2d(0.0, 0.0);
    this->P_ = Eigen::Matrix2d::Identity() * this->Confidence;
}


bool RLSLine::GetLineParams(double& a, double& b)
{
    if(this->PointCnt <= this->ThresholdPoint)
        return false;

    a = this->Now_a;
    b = this->Now_b;
    return true;
}