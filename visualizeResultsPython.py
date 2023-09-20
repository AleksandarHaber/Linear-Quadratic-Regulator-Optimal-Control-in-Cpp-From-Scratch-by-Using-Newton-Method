# -*- coding: utf-8 -*-
"""
This is the Python file used to visualize the results
and save the graphs

*/
"""

import numpy as np
import matplotlib.pyplot as plt

# Load the matrices and vectors from csv files 
# computed closed loop matrix
AclCpp = np.loadtxt("computedAcl.csv", delimiter=",")
# computed gain matrix
Kcpp = np.loadtxt("computedK.csv", delimiter=",")
# state trajectory
stateCpp= np.loadtxt("computedSimulatedStateTrajectory.csv", delimiter=",")
# solution Riccati
riccatiCpp= np.loadtxt("computedSolutionRiccati.csv", delimiter=",")

# plot the results
plt.figure(figsize=(8,8))
plt.plot(stateCpp.transpose(),linewidth=3, label='Controlled state')
plt.xlabel('time steps')
plt.ylabel('State')
plt.legend()
plt.savefig('controlledState.png',dpi=600)
plt.show()


