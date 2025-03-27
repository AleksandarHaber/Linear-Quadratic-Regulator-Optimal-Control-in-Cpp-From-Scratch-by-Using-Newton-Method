# Linear-Quadratic-Regulator-Optimal-Control-in-Cpp-From-Scratch-by-Using-Newton-Method

IMPORTANT NOTE: The code files are released under "Creative Commons Attribution-NonCommercial-NoDerivatives 4.0
International Public License (CC BY-NC-ND 4.0.)"  

**Brief explanation of the license (read the complete license):** 
**Attribution (BY):** You must give appropriate credit and reference to the creator and code (citation). You need to provide a link to the license and link to the code files. 
**NonCommercial (NC):** You may not use the material for commercial purposes. 
**NoDerivatives (ND):** You cannot remix, transform, or build upon the material, meaning you can only share the original work without any adaptations. If you plan to use the code for commercial purposes, contact the author at ml.mecheng@gmail.com



We implemented a solution of the Linear Quadratic Regulator (LQR) Optimal Control problem in C++. We use the Newton method to solve the Riccati equation and to compute the solution. The webpage tutorial explaining this implementation is given here:

https://aleksandarhaber.com/implementation-of-the-solution-of-the-linear-quadratic-regulator-lqr-control-algorithm-in-c-by-using-the-eigen-matrix-library/

Explanation of posted files:

- "LQRController.h"  - LQR class header file.
- "LQRController.cpp" - LQR class implementation file.
- "driver_code.cpp"  - driver code, you should start from here. By running this code file several CSV files will be generated. These files contain the computed LQR controller matrices and closed-loop state trajectories.
- "pythonLQR.py"  - Python code for computing the LQR controller by using the Control System Toolbox for Python
- "visualizeResultsPython.py" - this file is used to visualize the C++ state trajectories in Python and to load the computed LQR matrices into Python workspace such that they can be compared with the Python implementation.
