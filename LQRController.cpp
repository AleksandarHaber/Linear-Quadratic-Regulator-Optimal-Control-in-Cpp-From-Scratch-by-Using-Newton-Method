/*
Linear Quadratic Regulator (LQR) (Optimal) Controller implementation in C++
Author:Aleksandar Haber 
Date: September 2023 

We implemented a LQR control algorithm in C++ by using the Eigen C++ Matrix library 
The description of the LQR algorithm is given in this tutorial:

This is the implementation file of the LQR controller class

LICENSE: THIS CODE CAN BE USED FREE OF CHARGE ONLY FOR ACADEMIC AND EDUCATIONAL PURPOSES.
THAT IS, IT CAN BE USED FREE OF CHARGE ONLY IF THE PURPOSE IS NON-COMMERCIAL AND IF THE PURPOSE
IS NOT TO MAKE PROFIT OR EARN MONEY BY USING THIS CODE. 

IF YOU WANT TO USE THIS CODE IN THE COMMERCIAL SETTING, THAT IS, IF YOU WORK FOR A COMPANY OR IF YOU ARE INDEPENDENT
CONSULTANT AND IF YOU WANT TO USE THIS CODE, THEN WITHOUT MY PERMISSION AND WITHOUT PAYING THE PROPER FEE, YOU ARE NOT ALLOWED
TO USE THIS CODE. YOU CAN CONTACT ME AT 

ml.mecheng@gmail.com

TO INFORM YOURSELF ABOUT THE LICENSE OPTIONS AND FEES FOR USING THIS CODE.

*/
// C++ includes
#include <iostream>
#include<tuple>
#include<string>
#include<fstream>
#include<vector>
// Eigen header files
#include<Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/Eigenvalues> 
// LQR controller header file
#include "LQRController.h"


       /* 
        This constructor will initialize all the private variables, 
        and precompute some constants and matrices
        */
        // Input arguments"
        // Ainput, Binput - A and B system matrices 
        // Qinput - state weighting matrix
        // Rinput - control input weighting matrix
        // Sinput - state-input mixed weighting matrix
                
LQRController::LQRController(MatrixXd Ainput, MatrixXd Binput, 
                       MatrixXd Qinput, MatrixXd Rinput,MatrixXd Sinput)
{
    A=Ainput; B=Binput; Q=Qinput; R=Rinput; S=Sinput;
    n=A.rows(); m=B.cols();


    // create an empty LQR gain matrix
    K.resize(m,n); K.setZero();
    // create an empty closed-loop matrix Acl=A-B*K
    Acl.resize(n,n); Acl.setZero();
    // create an empty matrix for storing the solution of the Riccati equation
    solutionRiccati.resize(n,n);
    solutionRiccati.setZero();
    // identity matrix for computations
    In= MatrixXd::Identity(n,n);
    
    // simulatedStateTrajectory - this private matrix is initialized and resized by the function SimulateSystem()
    // this matrix is the simulated closed-loop state trajectory 
    // this state trajectory is again resized by the function SimulateSystem()
    
    
}

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

void LQRController::ComputeInitialGuess(MatrixXd &initialGuessMatrix)
{
    
    // here, we have to introduce new matrices in order to fit everything 
    // in the computational framework explained in the book
    MatrixXd Anew;
    MatrixXd Bnew;
    Anew=A-B*(R.inverse())*(S.transpose());
    Bnew=B*(R.inverse())*(B.transpose());
    
    // compute the eigenvalues of the open-loop matrix
    EigenSolver<MatrixXd> eigenValueSolver(Anew);
    // used to store the current eigenvalue
    complex<double> lambda; 
    // beta parameter from the book
    double bParam=0;
    // we store the real parts of the eigenvalues in this vector
    vector <double> realPartEigenValues;
    // extract the real parts of the eigenvalues and store them in the vector
    for (int i=0; i<n; i++)
    {
        lambda= eigenValueSolver.eigenvalues()[i];
        realPartEigenValues.push_back(lambda.real());
    }
    // compute the bParam
    bParam=(*min_element(realPartEigenValues.begin(), realPartEigenValues.end()));
    //cout<<bParam;
    bParam=(-1)*bParam;
    // be carefull about selecting this constant 0.02, you might need to change it if the convergence 
    // is not fast, 
    bParam=max(bParam,0.0)+0.02;
    // Abar is A+beta*I matrix from the equation 3.14 in the book
    MatrixXd Abar;
    // note that we need to transpose, since our Lyapunov solver is written like this
    // A^{T}X+XA=RHS
    // in the book, it is 
    // AX+XA^{T}=RHS 
    Abar=(bParam*In+Anew).transpose();
    // right-hand side of the equation 3.14 from the book
    MatrixXd RHS;
    RHS=2*Bnew*(Bnew.transpose());
    // solve the Lyapunov equation
    MatrixXd solutionLyapunov;
    solutionLyapunov.resize(n, n); 
    solutionLyapunov.setZero();
    // this residual is filled by the solution residual computed by the Lyapunov solver
    double residualLyap=10e10;
    // call the Lyapunov solver
    SolveLyapunovEquation(Abar, RHS, solutionLyapunov, residualLyap);
    // compute the initial guess matrix by using the equation from the book
    initialGuessMatrix=(Bnew.transpose())*(solutionLyapunov.inverse());
    // ensure that this solution is symmetric - this is a pure heuristic...
    initialGuessMatrix=0.5*(initialGuessMatrix+initialGuessMatrix.transpose());
    //cout<<endl<<"Initial guess of the solution:"<<endl;
    //cout<<endl<<initialGuessMatrix<<endl;
}

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
void LQRController::SolveLyapunovEquation(const MatrixXd &Am, const MatrixXd &  Rm, MatrixXd &SolutionMatrix, double &residual)
{
    // this is the left hand side after we vectorize the Lyapunov equation 
    // by using the Kronecker product
    MatrixXd LHSMatrix;
    // cout<<endl<<Rm<<endl;
    LHSMatrix=kroneckerProduct(In,Am.transpose())+kroneckerProduct(Am.transpose(),In);
    // this variable stores the solution as a vector that is later on resized to be a matrix
    MatrixXd solutionLyapunovVector;
    // this is the right-hand side after we vectorize the Lyapunov equation
    MatrixXd RHSVector;
    // vectorize the input right-hand size matrix
    RHSVector=Rm.reshaped(n*n,1);
    // compute the solution by solving a liner system
    // different options for computing the solution
    //solutionLyapunovVector=LHSMatrix.colPivHouseholderQr().solve(RHSVector);
    //solutionLyapunovVector=LHSMatrix.fullPivLu().solve(RHSVector);
    // solutionLyapunovVector=(LHSMatrix.inverse())*RHSVector;
     solutionLyapunovVector=LHSMatrix.completeOrthogonalDecomposition().solve(RHSVector);
    // reshape the solution vector to be the solution matrix
    SolutionMatrix=solutionLyapunovVector.reshaped(n,n);
    // just in case, make sure that the computed solution is symmetric, pure heuristics...
    SolutionMatrix=0.5*(SolutionMatrix+SolutionMatrix.transpose());
    // this is the residual matrix     
    MatrixXd residualMatrix;
    residualMatrix=(Am.transpose())*SolutionMatrix+SolutionMatrix*Am-Rm;
    // compute the residual by using the Frobenious norm
    residual=residualMatrix.squaredNorm();
}

/*
    This function computes the solution of the LQR problem.
    
    input arguments:

    - maxNumberIterations - maximum number of iterations
    - tolerance - relative convergence tolerance, for example, 1e6 


*/

void LQRController::ComputeSolution(int maxNumberIterations, double tolerance)
{
    // this is the initial guess of the solution
    MatrixXd initialGuess;
    initialGuess.resize(n, n); 
    initialGuess.setZero();
    // call the function for computing the initial guess
    ComputeInitialGuess(initialGuess);

    // check if the initial guess is stabilizing
    // this is very important, otherwise, the convergence cannot be guaranteed
    // initial gain matrix K
    MatrixXd initialK;
    // initial closed-loop matrix
    MatrixXd initialAcl;
    initialK=(R.inverse())*((B.transpose())*initialGuess+S.transpose());
    initialAcl=A-B*initialK;
    // here, we need to compute the eigenvalues of the initial closed loop matrix
    // to make sure that the initial solution is stabilizing
    EigenSolver<MatrixXd> eigenValueSolver(initialAcl);
    cout <<endl<< "The eigenvalues of the initial closed-loop matrix are:" << endl << eigenValueSolver.eigenvalues() << endl;
    cout <<"If all the eigenvalues are strictly in the left-half of the complex plane, the solution is stabilizing!"<<endl;

    // this is the private variable for storing the solution of the Riccati equation
    solutionRiccati= initialGuess;
    // in this matrix, we store the solution of the Lyapunov equation in every iteration
    MatrixXd solutionLyapunov;
    solutionLyapunov.resize(n, n); 
    solutionLyapunov.setZero();
    // matrix to store the right-hand side of the Lyapunov equation
    MatrixXd  RHS;
    // temporary matrix 
    MatrixXd  tmpMatrix;
    // this the update matrix that is used to compute the current relative error
    MatrixXd  updateMatrix;
    // this is the residual of the Lyapunov equation - the value is changed by the Lyapunov solver
    double residualLyap=10e10;
    // this variable is updated by the current value of the errorConvergence
    double errorConvergence=10e10;
    // this variable is used to track the current iteration of the while loop
    int currentIteration=0;
    while  (currentIteration<=maxNumberIterations && errorConvergence>=tolerance )
    {
        /* Approach from Mehrmann's book
         see page 90, of 
        Mehrmann, V. L. (Ed.). (1991). The autonomous linear quadratic control problem: theory and numerical solution. 
        Springer.
        */
        tmpMatrix=(B.transpose())*solutionRiccati+S.transpose();
        // LQR gain is the private variable
        K=(R.inverse())*tmpMatrix;
        // closed loop matrix is actually the private variable
        Acl=A-B*K;
        RHS=Q+(K.transpose())*R*K-S*K-(K.transpose())*(S.transpose());
        // Right-hand side of the Lyapunov equation should be with a minus sign...
        RHS=-RHS;
        // solve the Lyapunov equation
        SolveLyapunovEquation(Acl, RHS, solutionLyapunov, residualLyap);
        // this matrix is only used to track the error convergence
        updateMatrix=solutionLyapunov-solutionRiccati;
        // relative error is the ratio of 1-norms of updateLyapunov and solutionRiccati
        errorConvergence = (updateMatrix.cwiseAbs().colwise().sum().maxCoeff())/(solutionRiccati.cwiseAbs().colwise().sum().maxCoeff());
        // update the solution for the next iteration
        solutionRiccati=solutionLyapunov;
        // increment the iteration number
        currentIteration=currentIteration+1;
        //cout<<currentIteration<<endl;
    }
    // diagnostics
    // if we converged
    if (errorConvergence<tolerance)
    {
        cout<<endl<<"Solution computed within prescribed error tolerances!"<<endl;
        cout<<"Converged error: "<<errorConvergence<<endl;
        cout<<"Number of iterations:"<<currentIteration<<endl;
    }
    // if we did not converge
    if (currentIteration>maxNumberIterations)
    {
        cout<<endl<<"Maximum number of maximum iterations exceeded!"<<endl;
        cout<<"However, the current error is not below the error tolerance."<<endl;
        cout<<"Consider incresing the maximum number of iterations or decreasing the tolerances."<<endl;
        cout<<"Current error:"<<errorConvergence<<endl;
    }
   

    // compute the eigenvalues of the final computed closed loop matrix
    EigenSolver<MatrixXd> eigenValueSolver2(Acl);
    cout <<endl<< "The eigenvalues of the final closed-loop matrix: " << endl << eigenValueSolver2.eigenvalues() << endl;
    cout<<endl<<"Computed K matrix: "<<K<<endl;  
}
/*
    This function is used to simulate the closed loop system
    it will columns of the private member matrix simulatedStateTrajectory
    Input arguments: 
    - x0 - initial condition
    - simulationTimeSteps - total number of simulation time steps
    - h - discretization time step
*/
void LQRController::SimulateSystem(MatrixXd x0,int simulationTimeSteps, double h)
{
    simulatedStateTrajectory.resize(n,simulationTimeSteps);simulatedStateTrajectory.setZero();
    simulatedStateTrajectory.col(0)=x0;    

    // this is the discretized closed-loop matrix
    MatrixXd AclDiscrete;
    AclDiscrete=(In-h*Acl).inverse();
    simulatedStateTrajectory.col(0)=x0;

    for (int i=0; i<simulationTimeSteps-1; i++)
    {   
        simulatedStateTrajectory.col(i+1)=AclDiscrete*simulatedStateTrajectory.col(i);
    }
}

/*
         this function is used to save the variables as "csv" files
         adjust this function as you wish later on
         KFile - name of the file used to store the computed LQR gain matrix K
         AclFile  - name of the file used to store the closed loop matrix
         solutionRiccatiFile  - name of the file used to store the solution of the Riccati equation
         simulatedStateTrajectoryFile  - name of the file used to store the closed loop state trajectory
        */
void LQRController::SaveData(string KFile,string AclFile,string solutionRiccatiFile,string simulatedStateTrajectoryFile) const
{
	const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
	
	ofstream file1(KFile);
	if (file1.is_open())
	{
		file1 << K.format(CSVFormat);
		
		file1.close();
	}

	ofstream file2(AclFile);
	if (file2.is_open())
	{
		file2 << Acl.format(CSVFormat);
		file2.close();
	}
	
	ofstream file3(solutionRiccatiFile);
	if (file3.is_open())
	{
		file3 << solutionRiccati.format(CSVFormat);
		file3.close();
	}

	ofstream file4(simulatedStateTrajectoryFile);
	if (file4.is_open())
	{
		file4 << simulatedStateTrajectory.format(CSVFormat);
		file4.close();
	}

  
	
}
