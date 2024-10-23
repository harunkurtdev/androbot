% Define parameters
N = 1; % Example value, adjust as necessary
k_t = 1; % Spring constant
R_a = 1; % Radius of action
R = 1; % Wheel radius
g = 9.81; % Acceleration due to gravity
M_p = 1; % Mass of the chassis
M_RR = 1; % Mass of the right wheel
M_RL = 1; % Mass of the left wheel
J_RR = 1; % Inertia of the right wheel
J_RL = 1; % Inertia of the left wheel
J_P = 1; % Inertia of the chassis
J_Py = 1; % Inertia about the y-axis (added this)
L = 1; % Length
D = 1; % Width
b = 1; % Distance between wheels
f_dRR = 0; % Damping force on right wheel
f_dRL = 0; % Damping force on left wheel
f_dp = 0; % Damping force on chassis
f_c = 0; % Coulomb friction
H_TR = 0; % Torque on right wheel
H_TL = 0; % Torque on left wheel
C_R = 0; % Constant for right wheel
C_L = 0; % Constant for left wheel
V_R = 0; % Vertical force on right wheel
V_L = 0; % Vertical force on left wheel

% Calculate constants
k_1 = (N * k_t) / R_a;
k_2 = (N^2 * k_t^2) / (R * R_a);

% Define state variables
syms x_RR x_RL y_RR y_RL theta_P theta_y v_RR v_RL omega_P omega_y
syms theta_RR theta_RL H_R H_L V_TR V_TL
syms x_P y_P % Added chassis position variables

% Chassis dynamics
m = 2 * (M_p * J_RR * L^2 + M_p * M_RR * R^2 * L^2 + J_RR * J_P + J_P * M_RR * R^2);
n = 2 * J_Py * R^2 - J_RR * D^2 - M_RR * R^2 * L^2;

A_22 = (-2 * k_2 * (M_p * R * L^2 + M_p * R^2 * L + J_P * R)) / m;
A_23 = (-(M_p^2 * R^2 * L^2 * g)) / m;
A_42 = (-2 * k_2) / (J_P + M_p * L);
A_43 = (M_p * g * L) / (J_P + M_p * L);
A_66 = (-k_2 * D^2 * R) / n;

% Define A matrix
A = [0, 1, 0, 0, 0, 0;
     0, A_22, A_23, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, A_42, A_43, 0, 0, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, A_66];

% Define B matrix
B_21 = (k_1 * (M_p * R * L^2 + M_p * R^2 * L + J_P * R)) / m;
B_41 = k_1 / (J_P + M_p * L);
B_62 = (-k_1 * D * R) / n;

B = [0, 0;
     B_21, 0;
     0, 0;
     B_41, 0;
     0, 0;
     0, B_62];

% Display the matrices
disp('A matrix:');
disp(A);
disp('B matrix:');
disp(B);

% Define equations of motion for the right wheel
eq1 = J_RR * diff(theta_RR, 2) == M_p * g * L * theta_P + (M_RR * diff(x_RR, 2) + M_RL * diff(x_RL, 2)) * L - (f_dRR + f_dRL) * L + (J_RR * diff(theta_RR, 2) + J_RL * diff(theta_RL, 2)) * L / R - (C_R + C_L) * L / R;

% Define equations of motion for the left wheel
eq2 = M_RR * diff(x_RR, 2) == f_dRR + H_TR - H_R - b * diff(x_RR, 1) - f_c * sign(diff(x_RR, 1));
eq3 = M_RR * diff(y_RR, 2) == V_TR - (M_RR * g) - V_R;
eq4 = J_RR * diff(theta_RR, 2) == C_R - (H_TR - b * diff(x_RR, 1) - f_c * sign(diff(x_RR, 1))) * R;

% Define equations of motion for the left wheel
eq5 = M_RL * diff(x_RL, 2) == f_dRL + H_TL - H_L - b * diff(x_RL, 1) - f_c * sign(diff(x_RL, 1));
eq6 = M_RL * diff(y_RL, 2) == V_TL - M_RL * g - V_L;
eq7 = J_RL * diff(theta_RL, 2) == C_L - (H_TL - b * diff(x_RL, 1) - f_c * sign(diff(x_RL, 1))) * R;

% Chassis equations
eq8 = M_p * diff(x_P, 2) == f_dp + H_R + H_L;
eq9 = M_p * diff(y_P, 2) == V_R + V_L - M_p * g;
eq10 = J_P * diff(theta_P, 2) == (V_R + V_L) * L * sin(theta_P) - (H_R + H_L) * L - (C_L + C_R);
eq11 = J_P * diff(theta_y, 2) == (H_L - H_R) * D / 2;

% Display the equations
disp('Equations of motion for right wheel:');
disp(eq1);
disp('Equations of motion for left wheel:');
disp(eq5);
disp('Chassis equations:');
disp(eq8);
