% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%% A. Stablizing Controller Simulation: Study Cases
%   * Load disturbances with 5, 10, & 20 ms delays
%   * Vin disturbances with 5, 10, & 20 ms delays
%---------------------------------------------------------------------------------------------

%% ---- Case: Load Disturbance

clear all;close all;clc;format short g
set_current_path;

delays = [5 10 20]*1e-3;              % Communication delay (ms)
% delays = 5*1e-3;

simulation_name = 'Stablizing_TDS_Case';
simulation_type = 'Load Dist'

% Generate load disturbances with the delays
for tau = delays
    zero_ics = true;                  % Zero initial conditions
    sim_vin_dist = false;             % No Vin disturbance
    sim_load_dist = ~sim_vin_dist;    % Add Load disturbances

    design_stablizing_tds_controller    % Robust controller desing
    simulate_stablizing_tds_controller  % Simulation
end

%% ---- Case: Vin Disturbance 

clear all;close all;clc;format short g

delays = [5 10 20]*1e-3;              % communication delay vector (ms)
% delays = 5*1e-3;

simulation_name = 'Stablizing_TDS_Case';
simulation_type = 'Vin Dist'

% Generate load disturbances with the delays
for tau = delays
    zero_ics = true;                  % Zero initial conditions
    sim_vin_dist = true;              % Add Vin disturbance
    sim_load_dist = ~sim_vin_dist;    % No Load disturbances

    design_stablizing_tds_controller     % Robust controller desing
    simulate_stablizing_tds_controller   % Simulation
end

