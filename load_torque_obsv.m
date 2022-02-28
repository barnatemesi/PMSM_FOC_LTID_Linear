%% State-space model of the Luenberger observer
%Ini
% clc; clear all; close all;
s = tf('s');
%% Load torque observer, observer gain design
% System description
model_parameters;
tau_load = -1/1000; %0 %-1/1000

%Second-order mechanical system
%State is [omega_m; T_load]
A = [-B_m/J -1/J;
           0 tau_load];
% u = [T_mot]
B = [1/J;
     0];
% y = \hat{omega}_mot
C = [1 0]; %[1 1]
D = zeros(size(C, 1), size(B, 2));

sys1 = ss(A, B, C, D);
G = tf(sys1);
d_sys1 = c2d(sys1, ts, d_mode);

%% Designing observer gains
observability = [C; C*A];
rank(observability);

%Desired observer poles
const = 0.5; %2.5
P1 = const*[-100-100*j, -100+100*j]; %damping of ~0.707
% P1 = const*[-100-20*j, -100+20*j]; 
% P1 = const*[-100-60*j, -100+60*j]; 
% Observer gain
L = place(A', C', P1)'; % [200 -292] 
% Set the gains manually
% L1 = 100;
% L2 = -292;
% L = [L1 L2];
%Check the eigen values of the observer
eig_L_cont = eig(A - L*C);

%% Discrete Luenberger estimator
%Re-formulate the system matrices
A_e = A - L*C;
B_e = [B L];
C_e = eye(size(A_e));
D_e = zeros(size(C_e, 1), size(B_e, 2));

sys_est = ss(A_e, B_e, C_e, D_e);

eig_L_d = eig(A_e);
[d_A_e, d_B_e, d_C_e, d_D_e] = c2dm(A_e, B_e, C_e, D_e, ts, d_mode);

%% Kalman-filter
%Second-order mechanical system
%State is [omega_m; T_load]
A_KF = [-B_m/J -1/J;
           0 tau_load];
% u = [T_mot]
B_KF = [1/J;
        0];
% y = \hat{omega}_mot
C_KF = [1 0];
D_KF = zeros(size(C, 1), size(B, 2));

% x_dot = Ax + Bu + Gw
G = [1 0;
     0 1];

sys_KF = ss(A_KF, B_KF, C_KF, D_KF);
% Process noise
q_omega = 1/36^2;
q_load = 1/2^2;
q_est = diag([q_omega q_load]);
% Measurement noise
r_omega = 1/200^2;
r_est = diag([r_omega]);

% x = Ax + Bu + Gw
% y = Cx + Du + v
%Estimator gain
% lqe(a,g,c,q,r,nn)
K_obs = lqe(A_KF, G, C_KF, q_est, r_est);

%% Discrete Kalman-filter
%Re-formulate the system matrices
A_e_KF = A - K_obs*C;
B_e_KF = [B_KF K_obs];
C_e_KF = [1 0];
D_e_KF = zeros(size(C_e_KF, 1), size(B_e_KF, 2));

sys_KF2 = ss(A_e_KF, B_e_KF, C_e_KF, D_e_KF);

eig_L_d = eig(A_e);
[d_A_KF, d_B_KF, d_C_KF, d_D_KF] = c2dm(A_e_KF, B_e_KF, C_e_KF, D_e_KF, ts, d_mode);



