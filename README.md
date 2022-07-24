# PMSM_FOC_LTID_Linear
FOC control of a PMSM motor with Luenberger estimator and Kalman filter. The purpose of these is to estimate the load torque and see the effect of feedforward load-torque compensation during transients.
It was implemented in MATLAB/Simulink 2018b.

The estimated load is used for feedforward load-torque compensation.

References: 

Dan Simon - Optimal State Estimation

Z. Kuang, B. Du, S. Cui, and C. C. Chan, “Speed Control of Load Torque Feedforward Compensation Based on Linear Active Disturbance Rejection for Five-Phase PMSM”, IEEE Access, vol. 7, pp. 159 787–159 796, 2019, ISSN: 21693536. DOI: 10.1109/ACCESS.2019.2950368.

Initialization is partially done in: Model Properties / Callbacks / InitFcn*

Also; Run load_torque_obsv.m
