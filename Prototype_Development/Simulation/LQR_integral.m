clear all;

% Parameters
L = 1;
m = 6.85;
I = 0.93; % Moment of inertia
T = 0.445;
K = 9.18;
b1 = 0.0;
b2 = 0.0;
k1 = 31.37;
k2 = 0.869;

% Original State-space matrices
A = [0, 1, 0, 0, 0, 0;          
     -k1/m, -b1/m, 0, 0, 1/m, 1/m;
     0, 0, 0, 1, 0, 0;
     0, 0, -k2/I, -b2/I, L/(2*I), -L/(2*I);
     0, 0, 0, 0, -1/T, 0;
     0, 0, 0, 0, 0, -1/T];
B = [0, 0;
     0, 0;
     0, 0;
     0, 0;
     K/T, 0;
     0, K/T];
C = [1, 0, 0, 0, 0, 0; %  
     0, 0, 1, 0, 0, 0]; % 
D = zeros(size(C,1), size(B,2)); % Assume D matrix is zero

% Augmenting the state with integrators for each output
A_aug = [A, zeros(size(A,1), size(C,1)); 
         -C, zeros(size(C,1))];
B_aug = [B; zeros(size(C,1), size(B,2))];
C_aug = [C, zeros(size(C,1), size(C,1))];
D_aug = D;

% Redefine Q and R for the augmented system
Q_aug = diag([1, 0, 15, 0, 0, 0, 100, 100]); % Adjust weights as needed
R_aug = diag([8, 8]);

[K_aug, S_aug, E_aug] = lqr(A_aug, B_aug, Q_aug, R_aug);

% Define the reference input vector for both outputs
ref_input = [0.0; 2]; % Unit step response for both outputs

% Closed loop system using state feedback control with integral action
Ac_aug = A_aug - B_aug * K_aug;
Bc_aug = [zeros(size(A, 1), size(ref_input, 1)); eye(size(ref_input, 1))]; % Corrected
Cc_aug = C_aug;
Dc_aug = D_aug;

syscl_aug = ss(Ac_aug, Bc_aug, Cc_aug, Dc_aug);

% Time span for the simulation
t = 0:0.01:20; % Adjust the time range and step size as needed

% Simulate the closed-loop system response to the new reference input
[y_aug, t_aug, x_aug] = lsim(syscl_aug, ref_input * ones(1, length(t)), t);

% Calculate the control input to the plant
% Note: Since we are using an augmented system, the control input is calculated using the augmented state vector
u_control = -K_aug * x_aug';

% Plot the response of both outputs (from the augmented system)
figure;
subplot(3, 1, 1);
plot(t_aug, y_aug(:,1));
title('Response of Output 1 with Integral Control');
xlabel('Time (seconds)');
ylabel('Output 1');
grid on;

subplot(3, 1, 2);
plot(t_aug, y_aug(:,2));
title('Response of Output 2 with Integral Control');
xlabel('Time (seconds)');
ylabel('Output 2');
grid on;

% Plot the control inputs to the plant
subplot(3, 1, 3);
plot(t_aug, u_control');
title('Control Inputs to the Plant');
xlabel('Time (seconds)');
ylabel('Control Input');
legend('Input 1', 'Input 2');
grid on;
