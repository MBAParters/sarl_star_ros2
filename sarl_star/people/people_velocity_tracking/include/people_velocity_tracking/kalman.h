#pragma once

#include <vector>
#include <iostream>
#include <algorithm>

class Kalman
{
public:
    Kalman(double Q = 0.002, double R = 1.0, double P = 0.01);

    void update(const std::vector<double>& values);
    std::vector<double> values() const;

private:
    std::vector<double> x;  // State estimate (position, velocity, etc.)
    std::vector<double> p;  // Error covariance
    double Q;  // Process noise
    double R;  // Measurement noise
    double P;  // Initial error covariance
};