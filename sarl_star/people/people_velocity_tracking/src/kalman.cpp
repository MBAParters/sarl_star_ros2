#include "people_velocity_tracking/kalman.h"

Kalman::Kalman(double Q, double R, double P)
    : Q(Q), R(R), P(P)
{
}

void Kalman::update(const std::vector<double> &values)
{
    size_t N = values.size();

    if (x.empty())
    {
        x = values;
        p.resize(N, P);
    }
    else
    {
        for (size_t i = 0; i < N; ++i)
        {
            p[i] += Q;
            double k = p[i] / (p[i] + R);
            x[i] += k * (values[i] - x[i]);
            p[i] = (1 - k) * p[i];
        }
    }
}

std::vector<double> Kalman::values() const
{
    return x;
}