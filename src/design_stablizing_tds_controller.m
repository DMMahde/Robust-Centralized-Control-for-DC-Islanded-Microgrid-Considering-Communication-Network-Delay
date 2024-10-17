% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%% ---- III. TIME-DELAY CONTROL IN DC IMG
% A. STABILIZING CONTROLLER

% clear all;close all;clc;format short g
set_current_path;

% TABLE 1. System parameters of the DC ImG Fig 4.
dgu_parameter_calculation;

%% ---- DGU Toplogical Configuration
% FIGURE 4. DC ImG consisting of six DGUs
dcimg_network_configuration;

C = eye(18);        % Output matrix

% TABLE 2. Values for the paremeters usid in Theorems1, 2, and 3
% Stablizing Controller Parameters (d, epsilon, hi)
% hi = 1000 (see dgus_network_configuration)

epsilon = 1e-4;     % For 5, 10, & 20 ms delay  
d = 0.1;            % For 5, 10, & 20 ms delay  
% tau = 20*1e-3;    % Delay (ms)
h = tau;            % For simulation purposes
dt = 10*1e-5;       % Simulation time step
Tf = 1;             % Simulation end time (s)

%% ---- Theorem 1: Stablizing TDS controller
% Solving LMI to get the gians

[nx,nu] = size(B);
Pbar = sdpvar(nx);
P2bar = sdpvar(nx);
Sbar = sdpvar(nx);
Rbar = sdpvar(nx);
Qbar = sdpvar(nx);
S12bar = sdpvar(nx,nx,'full');
Y = sdpvar(nu,nx);

phi11bar = A*P2bar+P2bar'*A'+Sbar+Qbar-Rbar;

% LMI (8)
Phi_bar = [ phi11bar                        Pbar-P2bar+epsilon*P2bar'*A'            S12bar            B*Y+Rbar-S12bar;...
           (Pbar-P2bar+epsilon*P2bar'*A')' -epsilon*P2bar-epsilon*P2bar'+h^2*Rbar   zeros(nx)         epsilon*B*Y;...
            S12bar'                         zeros(nx)                             -(Sbar+Rbar)        Rbar-S12bar';...
           (B*Y+Rbar-S12bar)'              (epsilon*B*Y)'                          (Rbar-S12bar')'  -(1-d)*Qbar-2*Rbar+S12bar+S12bar'];

% LMI (9)
con_2 = [Rbar    S12bar;
         S12bar' Rbar];
Cons = [ Phi_bar <= 0 ] + [ con_2 >= 0 ] + [Pbar>=0] + [Sbar>=0] + [Rbar>=0] + [Qbar>=0];

optimize(Cons);

% sdpvar values
Pbarv = value(Pbar); P2barv = value(P2bar); Sbarv = value(Sbar); 
Rbarv = value(Rbar); Qbarv = value(Qbar); S12barv = value(S12bar); 
Yv = value(Y); phi11barv = A*P2barv+P2barv'*A'+Sbarv+Qbarv-Rbarv;

Phi_barv = value(Phi_bar);
con_2v = value(con_2);

K = Yv/P2barv
digits(2);
K_Regulator = vpa(K)
% save('data/K_Stablizing','K');

% Checking Feasibility

if max(eig(Phi_barv))< 0 && min(eig(con_2v))>=0 && min(eig(Pbarv))>=0 && min(eig(Sbarv))>=0 && min(eig(Rbarv))>=0 && min(eig(Qbarv))>=0
    fprintf('\n      Results are OK\n')
else
    fprintf('\n      Results are not OK\n')
end

% OL_poles = eig(A)
% CL_poles = eig([A+B*K])  
% Controller TF
Ireff = Vin./(((1-Dd).^2).*RL);

