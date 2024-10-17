% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Boost-converte parameters calculsed form Taxas Instruments Application 
% Report SLVA372C–November 2009–Revised January 2014.
% Equations reffered here are form the same report.
% The following four parameters are needed to calculate the power stage:
%   1. Input Voltage Range: VIN(min) and VIN(max)
%   2. Nominal Output Voltage: VOUT
%   3. Maximum Output Current: IOUT(max)

%clear all;close all;clc;format short g

Vin = [95 100 90 105 92 90];                % 2. Nominal input voltage
Vin_min = Vin*0.90;                         % 1. Min & Max input voltage
Vin_max = Vin*1.1;     

Vout = [381 380.5 380.2 379 379.5 380.7]; 
Vout_min = Vout*0.9; 
Vout_max = Vout*1.1; 
Pload = [2.5 2.0 1.8 2.5 3.0 2.5]*1e3;
Rload = Vout.^2./Pload;
Iout = Pload./Vout;                         % Calculating Iout normal
Iout_max = 5e3./Vout_min;                   % 3. Maximum current @ 5 kW

% Eq (1)
eta = .99;                                  % Converer efficency
D = 1 - (Vin)./Vout;                        % Duty cycle
Dmax = D + 0.1;    Dmin = D - 0.1;          % Uncertainity in D

fs = 25e3;                                  % PWM switching frequency

del_I_inductor = 0.4*Iout_max.*(Vout./Vin); % Eq (3)
L_min = Vin.*(Vout - Vin)./(del_I_inductor*fs.*Vout); % Eq (2)

del_Vout = 0.02*Vout;                       % ~ 7.5 Volts
Cout_min = Iout_max.*D./(fs*del_Vout);      % Eq (12)

Lt = L_min*2;
Ltmax = Lt*1.15;   Ltmin = Lt*0.70;             % Uncertainity in Lt

Ct = Cout_min*3;
Ctmax = Ct*1.15;   Ctmin = Ct*0.70;             % Uncertainity in Ct

RL = Rload;
Rt = [0.02 0.04 0.02 0.2 0.4 0.5];
Vref = Vout;

% The peak current that, inductor, switch, and diode has to withstand.
Isw_max = del_I_inductor./2 + Iout_max./(1-D); % Eq (4)

Rij = [0.0 0.5 2.0 0.0 0.0 10.0                 % R of nterconnection, Ohm
       0.5 0.0 0.0 4.0 0.0 0.0                  % Rji = R'ij
       2.0 0.0 0.0 4.0 0.0 0.0
       0.0 4.0 4.0 0.0 15.0 0.0
       0.0 0.0 0.0 15.0 0.0 4.0
       10.0 0.0 0.0 0.0 4.0 0.0];
% Rij = Rij+diag(RL);
Lij = [00  10 70 00 00 800                      % L of nterconnection, uH
       10  00 00 70 00 00                       % Lji = L'ij
       70  00 00 70 00 00
       00  70 70 00 25 00
       00  00 00 25 00 90
       800 00 00 00 90 00]*1e-6;
Vo = Vin./(1-D);

ESR1 = del_Vout./del_I_inductor;
ESR2 = del_Vout./((Iout_max./(1-Dmax))+(del_I_inductor/2));

%% ---- Load Variations 
% Pload
PLmax = Pload*1.35;                  % Uncertainity in Load
PLmax(1) = 3500;                    % Set to 3.5 kW
PLmin = Pload*0.70;                  % Uncertainity in Load
PLmin(4) = 800;                     % Set to 800 W
PLmin(5) = 1000;                     % Set to 800 W
RLmax = Vout.^2./PLmax;
RLmin = Vout.^2./PLmin;