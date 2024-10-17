% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%% ---- III. TIME-DELAY CONTROL IN DC IMG
% C. PREDICTOR-FEEDBACK BASED ROBUST CONTROLLER

% clear all;close all;clc;format short g
set_current_path;

% TABLE 1. System parameters of the DC ImG Fig 4.
dgu_parameter_calculation;

%% ---- DGU Toplogical Configuration
% FIGURE 4. DC ImG consisting of six DGUs
dcimg_network_configuration;

% Eq (19) Matrices C1, & D12
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

wu1 = 1;    % weight of the control input in penalty vector
wu2 = 1;    % weight of the control input in penalty vector
wu3 = 1;    % weight of the control input in penalty vector
wu4 = 1;    % weight of the control input in penalty vector
wu5 = 1;    % weight of the control input in penalty vector
wu6 = 1;    % weight of the control input in penalty vector

D12 = [zeros(3*N,N);diag([wu1,wu2,wu3,wu4,wu5,wu6])];

% TABLE 2. Values for the paremeters usid in Theorems 1, 2, and 3
% Predictor Robust Controller Parameters 
% (beta, u_bar, lamda, gama) (hi, wvii, wii, wei, wui)
% hi = 1000 (see dgus_network_configuration)

% tau =  5*1e-3;                    % Input time-delay (ms)
gama = 55;                         % Disturbance reduction parameter
lmda = 1;                          % Design parameter

Tf =   1;                          % final simulation time (s)
dt = 5e-5;                         % time step
h = tau;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---- Theorem 3: Predictor-based rfobust TDS controller
% Solving LMIs to get the gians

[nx,nu] = size(B);
nw = size(D,2);
nz = size(C1,1);
ny = size(C2,1);

Y = sdpvar(nu,nx);
X = sdpvar(nx);
M = expm(A*tau);    % Matrix exponential 

Psi_11 = X*A' + Y'*B' + A*X + B*Y;
Psi_22 = lmda*Psi_11;

% LMI Equation (22)
Psi = [ Psi_11      zeros(nx)       D                 B                X*C1'+Y'*D12';...
        zeros(nx)   Psi_22        lmda*M*D         zeros(nx,nu)        zeros(nx,nz);...
           D'       lmda*D'*M'   -gama^2*eye(nw)   zeros(nw,nu)        zeros(nw,nz);...
           B'       zeros(nu,nx)  zeros(nu,nw)    -(gama^2)*eye(nu)            D12';...
        C1*X+D12*Y  zeros(nz,nx)  zeros(nz,nw)       D12                  -eye(nz)];
     
% LMI for control effort reduction
beta2 = 2;
ubar = 1;           % TABLE 2. Maximum desirable control effort

% LMI Equation (23)
L_u = [beta2*lmda*ubar*X Y';...
       Y                 eye(nu)];

Cons= [X>=0] + [ Psi <= 0 ] + [L_u>=0];
% Cons= [X>=0] + [ Psi <= 0 ]; % Without control effor constrain

optimize(Cons);

% sdpvar values
Psiv=value(Psi); L_uv = value(L_u);
Yv=value(Y); Xv=value(X);
K = Yv/Xv
digits(2);
K_Predictor = vpa(K)

% save('K_Predictor','K');
% fid = fopen('K_Predictor.txt', 'wt');
% fprintf(fid,'% \n', vpa(K));
% fclose(fid);

% checking feasibility
if max(eig(Psiv))< 0 && min(eig(Xv))>=0 && min(eig(L_uv))>=0
% if max(eig(Psiv))< 0 && min(eig(Xv))>=0
    fprintf('\n      Results are O.K\n')
else
    fprintf('\n      Results are NOT O.K\n')
end


 % OL_poles = eig(A)
 % CL_poles = eig([A+B*K])  

% Controller TF

Ireff = Vin./(((1-Dd).^2).*RL);