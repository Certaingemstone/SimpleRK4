// rk4testclient.cpp : Double Pendulum Calculation

#include <iostream>
#include <fstream>
#include <cmath>
#include "rk4.h"

//Global pendulum properties
double M_1 = 1.0;
double M_2 = 1.0;
double L_1 = 1.0;
double L_2 = 1.0;
double GRAV = 9.8;

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
    double theta1, theta2;
    std::cout << "Enter first mass" << std::endl;
    std::cin >> M_1;
    std::cout << "Enter first arm length" << std::endl;
    std::cin >> L_1;
    std::cout << "Enter second mass" << std::endl;
    std::cin >> M_2;
    std::cout << "Enter second arm length" << std::endl;
    std::cin >> L_2;
    std::cout << "Enter first angle, in radians, from vertical" << std::endl;
    std::cin >> theta1;
    std::cout << "Enter second angle, in radians, from vertical" << std::endl;
    std::cin >> theta2;
    std::vector<double> stateInitial{ theta1, 0, theta2, 0 };

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
    if (!file.is_open()) { std::cerr << "Failed to open or create output file." << std::endl; }
    
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

