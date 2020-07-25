// rk4testclient.cpp : main

#include <iostream>
#include "rk4.h"

//Derivative functions (differential equations)
double x0prime(std::vector<double> state, double time) {
    return state[0];
}

double x1prime(std::vector<double> state, double time) {
    return state[1];
}

double x2prime(std::vector<double> state, double time) {
    return state[2];
}

int main()
{
    //Initial state
    std::vector<double> stateInitial{ 2, 3, 1 };

    //Create vector of differential equations
    std::vector<double (*)(std::vector<double>, double)> diffeqs;
    diffeqs.push_back(x0prime);
    diffeqs.push_back(x1prime);
    diffeqs.push_back(x2prime);

    //Initialize solver
    rk4::rk4Solve Solver(stateInitial, 0.0, 0.1, diffeqs);
    
    for (int i = 0; i < 10; i++) {
        Solver.iterate();
        for (double x : Solver.stateCurrent)
            std::cout << x << " ";
        std::cout << std::endl;
    }
}

