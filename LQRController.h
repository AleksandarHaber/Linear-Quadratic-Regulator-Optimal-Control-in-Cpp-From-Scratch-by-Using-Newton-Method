/*
Linear Quadratic Regulator (LQR) (Optimal) Controller implementation in C++
Author:Aleksandar Haber 
Date: September 2023 

We implemented a LQR control algorithm in C++ by using the Eigen C++ Matrix library 
The description of the LQR algorithm is given in this tutorial:

This is the header file of the LQR controller class

*/
#ifndef LQRCONTROLLER_H
#define LQRCONTROLLER_H

#include<string>
#include<Eigen/Dense>
using namespace Eigen;
using namespace std;


class LQRController{
    public:
        // default constructor - edit this later
        LQRController();

        /* 
        This constructor will initialize all the private variables, 
        and precompute some constants
        */
        // Input arguments"
        // Ainput, Binput - A and B system matrices 
        // Qinput - state weighting matrix
        // Rinput - control input weighting matrix
        // Sinput - state-input mixed weighting matrix
                
        LQRController(MatrixXd Ainput, MatrixXd Binput, 
                       MatrixXd Qinput, MatrixXd Rinput,MatrixXd Sinput);
        
                /*
        This function computes the initial guess of the solution of the Riccati equation
        by using the method explained in Section 3.3, page 95 of 
        D. A. Bini, B. Iannazzo, B. Meini - 
        "Numerical solution of algebraic Riccati equations", SIAM, (2012)
        NOTE: The initial guess of the solution should be a stabilizing matrix
        That is, the matrix A-B*K0 is a stable matrix, where K0 is the initial gain computed
        on the basis of the initial guess X0 of the solution of the Riccati equation.
        This function will compute X0.

        Input argument: initialGuessMatrix - this is a reference to the matrix that is used to store 
        the initial guess

        */

        void ComputeInitialGuess(MatrixXd &initialGuessMatrix);

        /*
        This function solves the Lyapunov equation by using the vectorization and Kronecker operator method 
        Am^{T}*SolutionMatrix+SolutionMatrix*A=Rm

        Input arguments:

            Am - input matrix A
            Rm - right hand size matrix Rm
            SolutionMatrix - the solution matrix, it is passed as a reference, that is the solution is 
            stored in this matrix 
            residual - this is the residual to track the accuracy, it is passed as a reference, that is,
            the computed residual is stored in this variable
            The residual is computed as the Frobenius norm of the residual matrix
            Am^{T}*SolutionMatrix+SolutionMatrix*A-Rm
        */
        void SolveLyapunovEquation(const MatrixXd &Am, const MatrixXd &  Rm, MatrixXd &SolutionMatrix, double &residual);
        
        // this function computes the solution by using the Newton algorithm
        void ComputeSolution(int numberIterations, double tolerance);

        /*
            This function is used to simulate the closed loop system
            it will columns of the private member matrix simulatedStateTrajectory
            Input arguments: 
            - x0 - initial condition
            - simulationTimeSteps - total number of simulation time steps
            - h - discretization time step
        */
        void SimulateSystem(MatrixXd x0,int simulationTimeSteps, double h);


        /*
         this function is used to save the variables as "csv" files
         adjust this function as you wish later on
         KFile - name of the file used to store the computed LQR gain matrix K
         AclFile  - name of the file used to store the closed loop matrix
         solutionRiccatiFile  - name of the file used to store the solution of the Riccati equation
         simulatedStateTrajectoryFile  - name of the file used to store the closed loop state trajectory
        */
        void SaveData(string KFile,string AclFile,string solutionRiccatiFile,string simulatedStateTrajectoryFile) const;


    private:

    // MatrixXd is an Eigen typdef for Matrix<double, Dynamic, Dynamic>
	    MatrixXd A,B; // system matrices
        MatrixXd Q,R,S;   // weighting matrices
	   
        MatrixXd K; // LQR control gain matrix 
        MatrixXd Acl; // LQR closed-loop matrix Acl=A-B*K
        MatrixXd solutionRiccati; // solution of the Riccati equation
        MatrixXd In; // identity matrix for computations
        MatrixXd simulatedStateTrajectory; // simulated state trajectory of the closed-loop state-space model

        unsigned int n; // state dimension 
        unsigned int m; // input dimension
      
    
};

#endif
