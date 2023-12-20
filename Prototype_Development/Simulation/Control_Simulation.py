import numpy as np
import matplotlib.pyplot as plt
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
K_aug, S_aug, E_aug = ctrl.lqr(A_aug, B_aug, Q_aug, R_aug)

# Define the reference input vector for both outputs
ref_input = np.array([[0.1], [2]])

# Closed loop system using state feedback control with integral action
Ac_aug = A_aug - B_aug.dot(K_aug)
Bc_aug = np.block([[np.eye(ref_input.shape[0])], [np.zeros((A.shape[0], ref_input.shape[0]))]])
Cc_aug = C_aug
Dc_aug = D_aug

syscl_aug = ctrl.ss(Ac_aug, Bc_aug, Cc_aug, Dc_aug)

# Time span for the simulation
t = np.linspace(0, 20, 2001)  # Adjust the time range and step size as needed

t_aug, y_aug = ctrl.forced_response(syscl_aug, T=t, U=ref_input * np.ones((ref_input.shape[0], len(t))))

# Calculate the control input to the plant
# Note that 'x_aug' is the state trajectory including both original and integrator states
error = ref_input - y_aug[:2, :]  # Error between reference and actual output
u_control = -K_aug[:, :2].dot(error)
print(u_control)
# Plot the response of both outputs (from the augmented system)
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t_aug, y_aug[0, :])
plt.title('Response of Output 1 with Integral Control')
plt.xlabel('Time (seconds)')
plt.ylabel('Output 1')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t_aug, y_aug[1, :])
plt.title('Response of Output 2 with Integral Control')
plt.xlabel('Time (seconds)')
plt.ylabel('Output 2')
plt.grid(True)

# Plot the control inputs to the plant
plt.subplot(3, 1, 3)
plt.plot(t_aug, u_control[0, :], label='Input 1')
plt.plot(t_aug, u_control[1, :], label='Input 2')
plt.title('Control Inputs to the Plant')
plt.xlabel('Time (seconds)')
plt.ylabel('Control Input')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
