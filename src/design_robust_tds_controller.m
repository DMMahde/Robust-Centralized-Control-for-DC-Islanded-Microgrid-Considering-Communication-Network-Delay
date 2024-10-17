% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%% ---- III. TIME-DELAY CONTROL IN DC IMG
% B. ROBUST CONTROLLER

% clear all;close all;clc;format short g
set_current_path;

% TABLE 1. System parameters of the DC ImG Fig 4.
dgu_parameter_calculation;

%% ---- DGU Toplogical Configuration
% FIGURE 4. DC ImG consisting of six DGUs
dcimg_network_configuration;

% Eq (12) Matrices C1, C2, & D12
% wvi, wii, wei, wui
wv1 = 0.6;       % weight of the voltage in penalty vector
wi1 = 0.1;       % weight of the current in penalty vector
we1 = 0.2;
C1_1 = diag([wv1, wi1, we1]);

wv2 = 0.6;       % weight of the voltage in penalty vector
wi2 = 0.1;       % weight of the current in penalty vector
we2 = 0.2;
C1_2 = diag([wv2, wi2, we2]);

wv3 = 0.6;       % weight of the voltage in penalty vector
wi3 = 0.1;       % weight of the current in penalty vector
we3 = 0.2;
C1_3 = diag([wv3, wi3, we3]);

wv4 = 0.6;       % weight of the voltage in penalty vector
wi4 = 0.1;       % weight of the current in penalty vector
we4 = 0.2;
C1_4 = diag([wv4, wi4, we4]);

wv5 = 0.6;       % weight of the voltage in penalty vector
wi5 = 0.1;       % weight of the current in penalty vector
we5 = 0.2;
C1_5 = diag([wv5, wi5, we5]);

wv6 = 0.6;       % weight of the voltage in penalty vector
wi6 = 0.1;       % weight of the current in penalty vector
we6 = 0.2;
C1_6 = diag([wv6, wi6, we6]);

C1 = [blkdiag(C1_1,C1_2,C1_3,C1_4,C1_6,C1_6);zeros(N,3*N)];

C2_1 = eye(3); C2_2 = eye(3); C2_3 = eye(3);
C2_4 = eye(3); C2_5 = eye(3); C2_6 = eye(3);
C2 = blkdiag(C2_1,C2_2,C2_3,C2_4,C2_5,C2_6);

% wui
wu1 = 1;    % weight of the control input in penalty vector
wu2 = 1;    % weight of the control input in penalty vector
wu3 = 1;    % weight of the control input in penalty vector
wu4 = 1;    % weight of the control input in penalty vector
wu5 = 1;    % weight of the control input in penalty vector
wu6 = 1;    % weight of the control input in penalty vector

D12 = [zeros(3*N,N); diag([wu1,wu2,wu3,wu4,wu5,wu6])];


% TABLE 2. Values for the paremeters usid in Theorems 1, 2, and 3
% Robust Controller Parameters 
% (d, epsilon, gamma) (hi, wvii, wii, wei, wui)
% hi = 1000 (see dgus_network_configuration)

% epsilon, d and h are selected randomly.
ep = 1e-4;          % For 5, 10, & 20 ms delay  
d = 0.1;            % For 5, 10, & 20 ms delay  
% tau = 20*1e-3;    % Delay (ms)
h = tau;            % For simulation purposes
dt = 10*1e-5;       % Simulation time step
Tf = 1;             % Simulation end time (s)

gamma = 13725;      % gamma = 13725

%% ---- Theorem 2: Robust TDS controller
% Solving LMIs to get the gians

[nx,nu]=size(B);
nw=size(D,2);
nz=size(C1,1);

Pbar = sdpvar(nx);
P2bar = sdpvar(nx);
Sbar = sdpvar(nx);
Rbar = sdpvar(nx);
Qbar = sdpvar(nx);
S12bar = sdpvar(nx,nx,'full');
Y = sdpvar(nu,nx);

phi11bar = A*P2bar+P2bar'*A'+Sbar+Qbar-Rbar;

% Phi_bar LMI (13)
Phi_bar = [ phi11bar                     Pbar-P2bar+ep*P2bar'*A'         S12bar             B*Y+Rbar-S12bar;...
           (Pbar-P2bar+ep*P2bar'*A')'    -ep*P2bar-ep*P2bar'+h^2*Rbar    zeros(nx)          ep*B*Y;...
            S12bar'                       zeros(nx)                      -(Sbar+Rbar)       Rbar-S12bar';...
           (B*Y+Rbar-S12bar)'             (ep*B*Y)'                       (Rbar-S12bar')'   -(1-d)*Qbar-2*Rbar+S12bar+S12bar'];
       
% Phi_12 and gma_I are partitions of LMI (11)
Phi_12 = [D            P2bar'*C1';...
          ep*D         zeros(nx,nz);...
          zeros(nx,nw) zeros(nx,nz);...
          zeros(nx,nw) Y'*D12'];
      
gma_I = [-(gamma^2)*eye(nw) zeros(nw,nz);...
          zeros(nz,nw)     -eye(nz)];

% Phi LMI (11)
Phi = [ Phi_bar  Phi_12;...
        Phi_12'  gma_I];

% Phi LMI (12)
con_2 = [Rbar    S12bar;
         S12bar' Rbar];
Cons=[ Phi <= 0 ] + [ con_2 >= 0 ] + [Pbar>=0] + [P2bar>=0] + [Sbar>=0] + [Rbar>=0];

optimize(Cons);

% sdpvar values

Pbarv=value(Pbar);P2barv=value(P2bar);Sbarv=value(Sbar);Rbarv=value(Rbar);
Qbarv=value(Qbar); S12barv=value(S12bar); Yv=value(Y);
phi11barv = A*P2barv+P2barv'*A'+Sbarv+Qbarv-Rbarv;
Phi_barv = value(Phi_bar);

Phiv = value(Phi);

con_2v = value(con_2);
K = Yv/P2barv

% checking feasibility

if max(eig(Phiv))< 0 && min(eig(con_2v))>=0 && min(eig(Pbarv))>=0 && min(eig(P2barv))>=0 && min(eig(Sbarv))>=0 && min(eig(Rbarv))>=0
    fprintf('\n      Results are O.K\n')
else
    fprintf('\n      Results are NOT O.K\n')
end


%  OL_poles = eig(A)
%  CL_poles = eig([A+B*K])  
% Controller TF

Ireff = Vin./(((1-Dd).^2).*RL);
