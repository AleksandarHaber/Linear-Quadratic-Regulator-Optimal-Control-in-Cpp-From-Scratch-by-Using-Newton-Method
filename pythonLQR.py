
"""
Linear Quadratic Regulator (LQR) in Python
This Python LQR code is used for comparison with the C++ code

@author: Aleksandar Haber 
date: 2023

pip install slycot   # optional
pip install control

"""

import matplotlib.pyplot as plt
import control as ct
import numpy as np



# masses, spring and damper constants
m1=2  ; m2=2   ; k1=100  ; k2=200 ; d1=1  ; d2=5; 
# define the continuous-time system matrices
A=np.array([[0, 1, 0, 0],
              [-(k1+k2)/m1 ,  -(d1+d2)/m1 , k2/m1 , d2/m1 ],
              [0 , 0 ,  0 , 1], 
              [k2/m2,  d2/m2, k2/m2, d2/m2]])
B=np.array([[0],[0],[0],[1/m2]])
C=np.array([[1, 0, 0, 0]])

D=np.array([[0]])

r=1; m=1 # number of inputs and outputs
n= 4 # state dimension


# define the state-space model
sysStateSpace=ct.ss(A,B,C,D)

#define the initial condition
x0=np.array([[0],[0],[0],[0]])

# print the system
print(sysStateSpace)


##############################################################
# design the LRQ controller
##############################################################


# state weighting matrix
Q=100*np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

# input weighting matrix
R=0.01*np.array([[1]])

# K - gain matrix 
# S - solution of the Riccati equation 
# E - closed-loop eigenvalues 

K, S, E = ct.lqr(sysStateSpace, Q, R)


Acl=A-np.matmul(B,K)

np.linalg.eig(Acl)[0]









