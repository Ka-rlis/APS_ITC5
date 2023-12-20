import numpy as np
import control as ctrl

# Parameters
L = 1
m = 6.85
I = 0.93  # Moment of inertia
T = 0.445
K = 9.18
b1 = 0.0
b2 = 0.0
k1 = 31.37
k2 = 0.869

# Original State-space matrices
A = np.array([[0, 1, 0, 0, 0, 0],
              [-k1/m, -b1/m, 0, 0, 1/m, 1/m],
              [0, 0, 0, 1, 0, 0],
              [0, 0, -k2/I, -b2/I, L/(2*I), -L/(2*I)],
              [0, 0, 0, 0, -1/T, 0],
              [0, 0, 0, 0, 0, -1/T]])
B = np.array([[0, 0],
              [0, 0],
              [0, 0],
              [0, 0],
              [K/T, 0],
              [0, K/T]])
C = np.array([[1, 0, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0]])
D = np.zeros((C.shape[0], B.shape[1]))

# Augmenting the state with integrators for each output
A_aug = np.block([[np.zeros((C.shape[0], C.shape[0])), -C],
                  [np.zeros((A.shape[0], C.shape[0])), A]])
B_aug = np.block([[np.zeros((C.shape[0], B.shape[1]))], [B]])
C_aug = np.block([[np.zeros((C.shape[0], C.shape[0])), C]])
D_aug = D

# Redefine Q and R for the augmented system
Q_aug = np.diag([10, 10, 1, 0, 15, 0, 0, 0])
R_aug = np.diag([20, 20])

# LQR controller for the augmented system
K_continous, S_aug, E_aug = ctrl.lqr(A_aug, B_aug, Q_aug, R_aug)

# Define the sampling time for discretization
sampling_time = 0.1  # in seconds

# Discretize the augmented system
sysd = ctrl.c2d(ctrl.ss(A_aug, B_aug, C_aug, D_aug), sampling_time, method='zoh')

# Define weight matrices for the LQR design
Q = np.diag([10, 10, 1, 0, 15, 0, 0, 0])
R = np.diag([20, 20])

# Solve for the optimal feedback gain matrix
K_discrete, _, _ = ctrl.dlqr(sysd.A, sysd.B, Q, R)

# Print the results
print("Continuous time K matrix: ", K_continous)
print("Discrete time K matrix: ", K_discrete)
