import matplotlib.dates as mdates
import numpy as np
import math
from numpy.core.arrayprint import dtype_is_implied
np.set_printoptions(suppress=True)

from numpy import genfromtxt
import matplotlib.pyplot as plt
import math
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy.utilities.codegen import codegen
init_printing(use_latex=True)

x0 = []
x1 = []
x2 = []

def prediction(X_hat_t_1,P_t_1,Q_t,drivingStraight):
	X_hat_t=X_hat_t_1

	if drivingStraight: # Driving straight
		X_hat_t[0] = X_hat_t_1[0] + X_hat_t_1[3]*dt * np.cos(X_hat_t_1[2])
		X_hat_t[1] = X_hat_t_1[1] + X_hat_t_1[3]*dt * np.sin(X_hat_t_1[2])
		X_hat_t[2] = X_hat_t_1[2]
		X_hat_t[3] = X_hat_t_1[3] + X_hat_t_1[5]*dt
		X_hat_t[4] = 0.0000001 #avoid numerical issues in Jacobians
		X_hat_t[5] = X_hat_t_1[5]

	else: # otherwise
		X_hat_t[0] = X_hat_t_1[0] + (X_hat_t_1[3]/X_hat_t_1[4]) * (np.sin(X_hat_t_1[4]*dt+X_hat_t_1[2]) - np.sin(X_hat_t_1[2]))
		X_hat_t[1] = X_hat_t_1[1] + (X_hat_t_1[3]/X_hat_t_1[4]) * (-np.cos(X_hat_t_1[4]*dt+X_hat_t_1[2])+ np.cos(X_hat_t_1[2]))
                #X_hat_t[2] = X_hat_t_1[2] + (X_hat_t_1[4]*dt + np.pi) % (2.0*np.pi) - np.pi
                #X_hat_t[3] = X_hat_t_1[3] + X_hat_t_1[5]*dt
                X_hat_t[4] = X_hat_t_1[4] # Constant Turn Rate
                X_hat_t[5] = X_hat_t_1[5] # Constant Acceleration
	# see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
        a13 = float((X_hat_t[3]/X_hat_t[4]) * (np.cos(X_hat_t[4]*dt+X_hat_t[2]) - np.cos(X_hat_t[2])))
        a14 = float((1.0/X_hat_t[4]) * (np.sin(X_hat_t[4]*dt+X_hat_t[2]) - np.sin(X_hat_t[2])))
        a15 = float((dt*X_hat_t[3]/X_hat_t[4])*np.cos(X_hat_t[4]*dt+X_hat_t[2]) - (X_hat_t[3]/X_hat_t[4]**2)*(np.sin(X_hat_t[4]*dt+X_hat_t[2]) - np.sin(X_hat_t[2])))
        a23 = float((X_hat_t[3]/X_hat_t[4]) * (np.sin(X_hat_t[4]*dt+X_hat_t[2]) - np.sin(X_hat_t[2])))
        a24 = float((1.0/X_hat_t[4]) * (-np.cos(X_hat_t[4]*dt+X_hat_t[2]) + np.cos(X_hat_t[2])))
        a25 = float((dt*X_hat_t[3]/X_hat_t[4])*np.sin(X_hat_t[4]*dt+X_hat_t[2]) - (X_hat_t[3]/X_hat_t[4]**2)*(-np.cos(X_hat_t[4]*dt+X_hat_t[2]) + np.cos(X_hat_t[2])))
        JA = np.matrix([[1.0, 0.0, a13, a14, a15, 0.0],
                    [0.0, 1.0, a23, a24, a25, 0.0],
                    [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
