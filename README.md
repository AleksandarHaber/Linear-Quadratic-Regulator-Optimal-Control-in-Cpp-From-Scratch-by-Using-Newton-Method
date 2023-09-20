# Linear-Quadratic-Regulator-Optimal-Control-in-Cpp-From-Scratch-by-Using-Newton-Method

We implemented a solution of the Linear Quadratic Regulator (LQR) Optimal Control problem in C++. We use the Newton method to solve the Riccati equation and to compute the solution. The webpage tutorial explaining this implementation is given here:

https://aleksandarhaber.com/implementation-of-the-solution-of-the-linear-quadratic-regulator-lqr-control-algorithm-in-c-by-using-the-eigen-matrix-library/

Explanation of posted files:

- "LQRController.h"  - LQR class header file.
- "LQRController.cpp" - LQR class implementation file.
- "driver_code.cpp"  - driver code, you should start from here. By running this code file several CSV files will be generated. These files contain the computed LQR controller matrices and closed-loop state trajectories.
- "pythonLQR.py"  - Python code for computing the LQR controller by using the Control System Toolbox for Python
- "visualizeResultsPython.py" - this file is used to visualize the C++ state trajectories in Python and to load the computed LQR matrices into Python workspace such that they can be compared with the Python implementation.
