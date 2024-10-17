% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%--------------------------------------------------------------------------------------------

clear all; close all; clc;
%% ---- Set Path
set_current_path;

%% ---- Time-Delay-System Simulations

% run('src/multi_case_simulation_tds_stablizing');
% run('src/multi_case_simulation_tds_robust');
% run('src/multi_case_simulation_tds_predictor');

%% ---- FIGURE 6:
i = 1;

figure('Name', 'FIGURE 6: TDS controller performance against load increase at $PCC_1$', 'NumberTitle','off')

subplot(2,2,1)
run('subplots/subplot_case_load_dist_tds_stablizing');

subplot(2,2,2)
run('subplots/subplot_case_load_dist_tds_robust');

subplot(2,2,3) % and subplot(2,2,4)
run('subplots/subplot_case_load_dist_tds_robust');

%% ---- FIGURE 7:
i = 5;

figure('Name', 'FIGURE 7: TDS controller performance against load decrease at $PCC_5$', 'NumberTitle','off')

subplot(2,2,1)
run('subplots/subplot_case_load_dist_tds_stablizing');

subplot(2,2,2)
run('subplots/subplot_case_load_dist_tds_robust');

subplot(2,2,3) % and subplot(2,2,4)
run('subplots/subplot_case_load_dist_tds_robust');


%% ---- FIGURE 8:
i = 2;

figure('Name', 'FIGURE 8: TDS controller performance against input-voltage variations for $PCC_2$', 'NumberTitle','off')

subplot(2,2,1)
run('subplots/subplot_case_vin_dist_tds_stablizing');

subplot(2,2,2)
run('subplots/subplot_case_vin_dist_tds_robust');

subplot(2,2,3) % and subplot(2,2,4)
run('subplots/subplot_case_vin_dist_tds_predictor');
