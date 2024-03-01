# Linear-Quadratic-Regulator-Optimal-Control-in-Cpp-From-Scratch-by-Using-Newton-Method

FIRST, READ THE LICENSE AT THE END OF THIS FILE.

We implemented a solution of the Linear Quadratic Regulator (LQR) Optimal Control problem in C++. We use the Newton method to solve the Riccati equation and to compute the solution. The webpage tutorial explaining this implementation is given here:

https://aleksandarhaber.com/implementation-of-the-solution-of-the-linear-quadratic-regulator-lqr-control-algorithm-in-c-by-using-the-eigen-matrix-library/

Explanation of posted files:

- "LQRController.h"  - LQR class header file.
- "LQRController.cpp" - LQR class implementation file.
- "driver_code.cpp"  - driver code, you should start from here. By running this code file several CSV files will be generated. These files contain the computed LQR controller matrices and closed-loop state trajectories.
- "pythonLQR.py"  - Python code for computing the LQR controller by using the Control System Toolbox for Python
- "visualizeResultsPython.py" - this file is used to visualize the C++ state trajectories in Python and to load the computed LQR matrices into Python workspace such that they can be compared with the Python implementation.

LICENSE: THIS CODE CAN BE USED FREE OF CHARGE ONLY FOR ACADEMIC AND EDUCATIONAL PURPOSES. THAT IS, IT CAN BE USED FREE OF CHARGE ONLY IF THE PURPOSE IS NON-COMMERCIAL AND IF THE PURPOSE IS NOT TO MAKE PROFIT OR EARN MONEY BY USING THIS CODE.

IF YOU WANT TO USE THIS CODE IN THE COMMERCIAL SETTING, THAT IS, IF YOU WORK FOR A COMPANY OR IF YOU ARE AN INDEPENDENT
CONSULTANT AND IF YOU WANT TO USE THIS CODE, THEN WITHOUT MY PERMISSION AND WITHOUT PAYING THE PROPER FEE, YOU ARE NOT ALLOWED TO USE THIS CODE. YOU CAN CONTACT ME AT

aleksandar.haber@gmail.com

TO INFORM YOURSELF ABOUT THE LICENSE OPTIONS AND FEES FOR USING THIS CODE.
ALSO, IT IS NOT ALLOWED TO 
(1) MODIFY THIS CODE IN ANY WAY WITHOUT MY PERMISSION.
(2) INTEGRATE THIS CODE IN OTHER PROJECTS WITHOUT MY PERMISSION.

 DELIBERATE OR INDELIBERATE VIOLATIONS OF THIS LICENSE WILL INDUCE LEGAL ACTIONS AND LAWSUITS. 
