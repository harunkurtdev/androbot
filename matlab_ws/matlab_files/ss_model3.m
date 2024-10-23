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
J_Py = 1; % Inertia about the y-axis
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

% Define target position and parameters for PID control
x_target = 5; % Target x position
Kp_x = 2; % Proportional gain for x
Kd_x = 1; % Derivative gain for x

% Simulation parameters
dt = 0.01; % Time step
t_end = 10; % Total time
time = 0:dt:t_end;

% Initial state variables
x_P = 0; % Initial x position of chassis
y_P = 0; % Initial y position of chassis
theta_P = 0; % Initial orientation of chassis
v_P = 0; % Initial velocity of chassis

% Simulation loop
for t = time
    % Calculate errors
    e_x = x_target - x_P; % Position error
    de_x = -v_P; % Velocity error (assuming v_P is the current velocity)

    % Control input for x position
    u_x = Kp_x * e_x + Kd_x * de_x; 

    % Update state variables (simple dynamics model)
    x_P = x_P + v_P * dt; % Update x position
    v_P = v_P + u_x * dt; % Update velocity based on control input

    % Display current position
    fprintf('Time: %.2f, x_P: %.2f, v_P: %.2f\n', t, x_P, v_P);
    
    % Here, you can update other state variables if needed based on your dynamics
end

disp('Simulation finished.');
