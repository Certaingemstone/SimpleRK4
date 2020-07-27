// rk4testclient.cpp : Double Pendulum Calculation

#include <iostream>
#include <fstream>
#include <cmath>
#include "rk4.h"

//Global pendulum properties
constexpr double M_1 = 2.0;
constexpr double M_2 = 1.0;
constexpr double L_1 = 3.0;
constexpr double L_2 = 2.0;
constexpr double GRAV = 9.8;

//Derivative functions (differential equations) from point mass double pendulum Lagrangian
//All angular units in radians
//x0 is angle from vertical of first point mass
//x1 is the time derivative of x0 (x0prime)
//x2 is angle from vertical of second point mass
//x3 is the time derivative of x2 (x2prime)
double x0prime(std::vector<double> x, double time) {
    return x[1];
}

double x1prime(std::vector<double> x, double time) {
    //time derivative of x1 is acceleration of x0; from equations of motion
    double Delta = x[0] - x[2];
    double top = M_2 * GRAV * sin(x[2]) * cos(Delta) - M_2 * L_1 * x[1] * x[1] * sin(Delta) * cos(Delta) - M_2 * L_2 * x[3] * x[3] * sin(Delta) - (M_1 + M_2) * GRAV * sin(x[0]);
    double bottom = (M_1 + M_2) * L_1 - M_2 * L_1 * cos(Delta) * cos(Delta); 
    return top/bottom;
}

double x2prime(std::vector<double> x, double time) {
    return x[3];
}
double x3prime(std::vector<double> x, double time) {
    //time derivative of x3 is acceleration of x2; from equations of motion
    double Delta = x[0] - x[2];
    double top = ((M_1 + M_2) * (L_1 * x[1] * x[1] * sin(Delta) + GRAV * sin(x[0]) * cos(Delta) - GRAV * sin(x[2]))) + M_2 * L_2 * x[3] * x[3] * sin(Delta) * cos(Delta);
    double bottom = M_1*L_2 + M_2*L_2 - M_2*L_2*cos(Delta)*cos(Delta);
    return top/bottom;
}

int main()
{
    //Initial state
    std::vector<double> stateInitial{ 2, 1, 2, 1 };

    //Create vector of differential equations
    std::vector<double (*)(std::vector<double>, double)> diffeqs;
    diffeqs.push_back(x0prime);
    diffeqs.push_back(x1prime);
    diffeqs.push_back(x2prime);
    diffeqs.push_back(x3prime);

    //Initialize solver
    rk4::rk4Solve Solver(stateInitial, 0.0, 0.02, diffeqs);

    //Output file
    std::fstream file;
    file.open("angles.csv", std::fstream::out);
    
    for (int i = 0; i < 1000; i++) {
        Solver.iterate();
        for (double x : Solver.stateCurrent) {
            std::cout << x << " ";
            file << x << ",";
        }
        file << "\n";
        std::cout << std::endl;
    }

    file.close();
    return 0;
}

