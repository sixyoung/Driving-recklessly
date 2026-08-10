#ifndef QUINTICPOLYNOMIAL_H
#define QUINTICPOLYNOMIAL_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>

class QuinticPolynomial
{
public:
    QuinticPolynomial() = default;
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double t);
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
    double getSmax();

private:
    double a0, a1, a2, a3, a4, a5;
    double Max_s;
};

#endif // QUINTICPOLYNOMIAL_H
