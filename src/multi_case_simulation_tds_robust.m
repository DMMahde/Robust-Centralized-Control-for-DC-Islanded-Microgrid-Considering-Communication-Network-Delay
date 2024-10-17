% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%% B. Robust Controller Simulation: Study Cases
%   * Load disturbances with 5, 10, & 20 ms delays
%   * Vin disturbances with 5, 10, & 20 ms delays
%---------------------------------------------------------------------------------------------

%% ---- Case: Load Disturbance

clear all;close all;clc;format short g
set_current_path;

delays = [5 10 20]*1e-3;             % Communication delay (ms)
% delays = 5*1e-3;

simulation_name = 'Robust_TDS_Case';
simulation_type = 'Load Dist'

% Generate load disturbances with the delays
for tau = delays
    zero_ics = true;                  % Zero initial conditions
    sim_vin_dist = false;             % No Vin disturbance  
    sim_load_dist = ~sim_vin_dist;    % Add Load disturbances

    design_robust_tds_controller     % Robust controller desing
    simulate_robust_tds_controller   % Simulation
end

%% Predictor-based H_infinty Controller: Vin Disturbance Case

clear all;close all;clc;format short g

delays = [5 10 20]*1e-3;            % Communication delay (ms)
% delays = 5*1e-3;

simulation_name = 'Robust_TDS_Case';
simulation_type = 'Vin Dist'

% Generate load disturbances with the delays
for tau = delays
    zero_ics = true;                 % Zero initial conditions
    sim_vin_dist = true;             % Add Vin disturbance  
    sim_load_dist = ~sim_vin_dist;   % Add Load disturbances

    design_robust_tds_controller     % Robust controller desing
    simulate_robust_tds_controller   % Simulation
end
