#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_S(w, q):
	v = -np.cross(w, q)
	S = np.array([[0, -w[2], w[1], v[0]], [w[2], 0, -w[0], v[1]], [-w[1], w[0], 0, v[2]], [0, 0, 0, 0]])
	return S

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	q1 = np.array([-0.15, 0.15, 0.01])
	q2 = np.array(np.add(q1, [0, 0.120, 0.152]))
	q3 = np.array(np.add(q2,[0.244, 0, 0]))
	q4 = np.array(np.add(q3, [0.213, -0.093, 0]))
	q5 = np.array(np.add(q4, [0, 0.083, 0]))
	q6 = np.array(np.add(q5, [0.083, 0, 0]))
	r = np.add(q6, np.array(np.add([0, 0.082, 0], [0, 0.059, 0.0535])))
	
	w1 = np.array([0, 0, 1])
	w2 = np.array([0, 1, 0])
	w3 = np.array([0, 1, 0])
	w4 = np.array([0, 1, 0])
	w5 = np.array([1, 0, 0])
	w6 = np.array([0, 1, 0])
	S = np.array([Get_S(w1, q1), Get_S(w2, q2), Get_S(w3, q3), Get_S(w4, q4), Get_S(w5, q5), Get_S(w6, q6)])
	print(r)
	M = np.array([[0, -1, 0, r[0]], [0, 0, -1, r[1]], [1, 0, 0, r[2]], [0, 0, 0, 1]])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]
	
	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	[M, S] = Get_MS()

	#T = expm(S[0]*theta1)*expm(S[1]*theta2)*expm(S[2]*theta3)*expm(S[3]*theta4)*expm(S[4]*theta5)*expm(S[5]*theta6)*M
	T = np.matmul(expm(S[0]*theta1), np.matmul(expm(S[1]*theta2), np.matmul(expm(S[2]*theta3), np.matmul(expm(S[3]*theta4), np.matmul(expm(S[4]*theta5), np.matmul(expm(S[5]*theta6), M))))))
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
