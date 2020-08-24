# SimpleRK4
C++ implementation of a 4th-order Runge-Kutta numerical solver for systems of 1st order differential equations. 

# Description
Accepts arbitrary number and type of equations as input, returns vector of variable values over time; useful for state-space representations of dynamical systems. 

# How to use
After including the files (`rk4.h` and the accompanying implementations in `rk4.cpp`), declare your differential equations as functions accepting a `vector<double>` (state of the system) and a `double` (time), while returning a `double`. For example,
```cpp
    double dxdt(std::vector<double> state, double time) {
        double derivative;
        //some code here to calculate the derivative of x given the state and/or time
        return derivative;
    }
```
Then in the main function, create a vector of function pointers to store your derivatives, like
```cpp
    std::vector<double (*)(std::vector<double>, double)> diffeqs;
    diffeqs.push_back(dxdt);
    diffeqs.push_back(dydt);
```
Also create a `vector<double>` to store your initial state.
    
Now, instantiate an `rk4::rk4Solve` with your initial state vector, starting time, size of time step, and derivative functions (note the number of state variables and number of equations governing them must match):
```cpp
    rk4::rk4Solve Solver(stateInitial, 0.0, 0.02, diffeqs);
```
Calling `iterate()` will move the system forward by one timestep, returning a vector of the new state variables.

See demo code for details, in this case solving the equations of motion of a double pendulum.


 
