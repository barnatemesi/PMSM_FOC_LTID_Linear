%% Load Estimator Designs
%Ini
% clc; clear all; close all;
s = tf('s');
%% Load torque observer, observer gain design
% System description
model_parameters;
tau_load = 0; %0 %-1/1000

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
G_sys1 = tf(sys1);
d_sys1 = c2d(sys1, ts, 'tustin'); %'zoh' %'tustin'

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
B_e = [B L]; %D = 0
C_e = eye(size(A_e));
D_e = zeros(size(C_e, 1), size(B_e, 2));

sys_est = ss(A_e, B_e, C_e, D_e);

eig_L = eig(A_e);
[d_A_e, d_B_e, d_C_e, d_D_e] = c2dm(A_e, B_e, C_e, D_e, ts, d_mode);

%% Kalman-filter
%Second-order mechanical system
%State is [ omega_m, T_load ]
A_KF = [-B_m/J -1/J;
           0 tau_load];
% u = [ T_mot ]
B_KF = [1/J;
        0];
% y = [ omega_m ]
% v = [ v_omega ]
C_KF = [1 0];
D_KF = zeros(size(C_KF, 1), size(B_KF, 2));

% x_dot = Ax + Bu + Gw
G = eye(size(A_KF, 1));

sys_KF = ss(A_KF, B_KF, C_KF, D_KF);
% Discretization of the dynamic system
[ d_A_KF, d_B_KF, d_C_KF, d_D_KF ] =...
    c2dm( A_KF, B_KF, C_KF, D_KF, ts, 'tustin' ); %'zoh' %d_mode %'tustin'
d_sys_KF = ss(d_A_KF, d_B_KF, d_C_KF, d_D_KF);

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
% eig(A_KF - K_obs*C_KF)

%% Discrete Kalman-filter
%Re-formulate the system matrices
A_e_KF = A_KF - K_obs * C_KF;
B_e_KF = [B_KF K_obs]; %D_KF = 0
C_e_KF = eye(size(A_e_KF));
D_e_KF = zeros(size(C_e_KF, 1), size(B_e_KF, 2));

sys_KF2 = ss(A_e_KF, B_e_KF, C_e_KF, D_e_KF);

eig_KF = eig(A_e_KF);
[d_A_KF, d_B_KF, d_C_KF, d_D_KF] = c2dm(A_e_KF, B_e_KF, C_e_KF, D_e_KF, ts, d_mode);

%% Kalman-filter, Colored (pink) noise
% The point is: the measurement noise v_omega is converted into a process
% noise now. Tuned by w_white_noise
tau_noise = -1 / 0.0006;
% Augment the original estimator system matrices!
% d_nu = -1 / tau_noise * v + xi, xi - white noise, v - colored noise
% State is [ omega_m, T_load nu_est ]
A_KF_col = [A_KF zeros(2,1); 0 0 tau_noise];
B_KF_col = [B_KF; 0];
C_KF_col = [1 0 1];
D_KF_col = zeros(size(C_KF_col, 1), size(B_KF_col, 2));
G_col = eye(size(A_KF_col, 1));

sys_KF_col = ss(A_KF_col, B_KF_col, C_KF_col, D_KF_col);
% Discretization of the dynamic system
[ d_A_KF_col, d_B_KF_col, d_C_KF_col, d_D_KF_col ] =...
    c2dm( A_KF_col, B_KF_col, C_KF_col, D_KF_col, ts, 'zoh' ); %'zoh' %d_mode %'tustin'
d_sys_KF_col = ss(d_A_KF_col, d_B_KF_col, d_C_KF_col, d_D_KF_col);

% Process noise
q_omega = 1/36^2;
q_load = 1/2^2;
q_omega_process = q_omega * 10^4;
q_est_col = diag([q_omega q_load q_omega_process]);
% Measurement noise
r_omega = 1/200^2; %previous one
r_omega_mod = 1e-7;
r_est_col = diag([r_omega_mod]);

%Estimator gain
K_obs_col = lqe(A_KF_col, G_col, C_KF_col, q_est_col, r_est_col);

%Re-formulate the system matrices
A_e_KF_col = A_KF_col - K_obs_col * C_KF_col;
B_e_KF_col = [B_KF_col K_obs_col]; %D_KF = 0
C_e_KF_col = eye(size(A_e_KF_col));
D_e_KF_col = zeros(size(C_e_KF_col, 1), size(B_e_KF_col, 2));

sys_KF_col2 = ss(A_e_KF_col, B_e_KF_col, C_e_KF_col, D_e_KF_col);

eig_KF_col = eig(A_e_KF_col);
[d_A_KF_col, d_B_KF_col, d_C_KF_col, d_D_KF_col] =...
    c2dm(A_e_KF_col, B_e_KF_col, C_e_KF_col, D_e_KF_col, ts, d_mode);

