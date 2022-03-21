% Siemens PMSM ROTEC 1FT6084-8SH7
nrat             = 4500;              % Rated speed [rpm]
Npp              = 4;                 % Number of pole pairs = 1/2 number of poles
Omegae_rat       = 2*pi*nrat/60*Npp;  % Rated electrical angular frequency
Lndmpm           = 0.12258;           % Rotor peak PM flux linkage [Web.turns],
Rs               = 0.268*1;           % [Ohm] Rs = system resistance = machine resistance, cable resistance, and inverter resistance. 
Ld               = 2.2e-3;            % [H] d-axis inductance 
Lq               = 2.2e-3;            % [H] q-axis inductance 
% The motor is a surface mounted PMSM
L_pm = Lq;
L_q = L_pm;
L_s = L_pm;
f                =-Rs/L_pm;
g                =1/L_pm;
Vs               = 360;               % Max. RMS phase voltage          
Vph              = Vs*sqrt(2);        % Peak value of phase voltage [V]                          

% Limiting the max. transient current in the output of speed loop PI
I_rated          = 18;                % Peak rated power                 
Iphmax           = I_rated*sqrt(2)*3;

% Mechanical system
Kt               = 1.01;              % Torque constant [Nm/A]           
J                = 0.0146;            % Total inertia: J_induction motor+J_PMSM+J_coupling [J.m^2]
B_m              = 0.0016655;         % Viscous friction [Ns/m]
C_m              = 0.2295;            % Columb friction [Nm]
gamma            = 0.001;             % constant for tanh function for columb friction
Trat             = 14;                % Rated torque [Nm]
Tl_const        = Trat/nrat^2;        % Fan load torque constants

% Danfoss FC302 VSI 15 kW
fs               = 1*5e3;             % switching frequency of inverter [kHz]
ts               = 1/fs;
d_mode           = 'tustin';
t_dead           = 2.5*10^(-6);       % Dead Time [sec]

% Initilizing the Simulink model
Omegae_ini       = 0;                 % Initial motor shaft speed, electrical value, [rad]
Lndd_ini         = Lndmpm;            % This means at t=0, theta=0 and N-pole aligned with d-axis 

% Limits from lab model
i_maxim_PM       = 35;                % maximum current of PMSM [A]
Vdq_limit        = 300;
T_PM_limit       = 14;                % load torque limit