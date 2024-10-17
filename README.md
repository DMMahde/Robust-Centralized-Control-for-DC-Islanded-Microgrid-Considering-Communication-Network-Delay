# Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
[IEEE Access, *Vol: 9*, *April 23, 2020*](https://doi.org/10.1109/ACCESS.2020.2989777)

# Overview

![The San Juan Mountains are beautiful!](/assets/images/san-juan-mountains.jpg "San Juan Mountains")

## Abstract
In recent years, the application of renewable energy resources (RES) with DC output has increased, and RES integration as DC islanded microgrids (DC ImGs) has attracted the attention of many researchers. However, DC ImGs face many challenges, and voltage stability is extremely critical for efficient power distribution. This challenge becomes more prominent when exogenous disturbances, as well as time-delay, exist in the system mainly because of the communication network. In this study, we develop a mathematical model of the time-delay DC ImG. To compensate for the effect of the time-delay, three control strategies are introduced-stabilizing, robust, and robust-predictor. The controller's stability is guaranteed based on the Lyapunov-Krasovskii theorem, whereas for the exogenous disturbance rejection, the $\mathcal(L)_2$ -norm of the system is reduced. Furthermore, to obtain the proposed controllers' gains, linear-matrix-inequality constraints are formulated. The performances of the controllers are investigated through numerous simulations, and a detailed analysis is presented.

## Getting Started
[YALMIP](https://yalmip.github.io/) and [MOSEK](https://www.mosek.com/) tolbox for [MATLAB](https://www.mathworks.com/?s_tid=gn_logo) is required to run the code.
* Run `yalmiptest` to check correct installation of YALMIP.
* Run `mosekdiag` to check correct installation of MOSEK.

## Code Structure
* `simulate_and_plot_all_cases.m`: Main simulation file for the research paper.
* `set_current_path.m`: To smoothly run MATLAB files without having path issues.
### src
* `design_stablizing_tds_controller.m`: Designing the *stablizing* controller (III. A. paper)l
* `design_robust_tds_controller.m`: Designing the *robust* controller (III. B. paper).
* `design_predictor_tds_controller.m`: Designing the *predictor-feedback robust* controller (III. C. paper).
* `simulate_stablizing_tds_controller.m`: Single case simulation setup.
* `simulate_robust_tds_controller.m`: Single case simulation setup.
* `simulate_predictor_tds_controller.m`: Single case simulation setup.
* `multi_case_simulation_tds_stablizing.m`: Multi case simulation setup.
* `multi_case_simulation_tds_robust.m`: Multi case simulation setup.
* `multi_case_simulation_tds_predictor.m`: Multi case simulation setup.
* `dgu_parameter_calculation.m`: *DGU* parameters calculation (TABLE 1. paper).
* `dcimg_network_configuration.m`: *DC ImG* with six DGUs, network configuration (FIGURE 4. paper).
* `set_current_path.m`: To smoothly run MATLAB files without having path issues.
### subplots
* `subplot_case_load_dist_tds_stablizing.m`: A subplot of FIGURE 6 & 7.
* `subplot_case_load_dist_tds_robust.m`: A subplot of FIGURE 6 & 7.
* `subplot_case_load_dist_tds_predictor.m`: A subplot of FIGURE 6 & 7.
* `subplot_case_vin_dist_tds_stablizing.m`: A subplot of FIGURE 8.
* `subplot_case_vin_dist_tds_robust.m`: A subplot of FIGURE 8.
* `subplot_case_vin_dist_tds_predictor.m`: A subplot of FIGURE 8.
* `set_current_path.m`: To smoothly run MATLAB files without having path issues.
### data
#### vin_signal
* `VinSignals_1min_5e5.mat`: Pre-generated Vin_i disturbances signals with dt = 5e-5.
* `VinSignals_1min_10e5.mat`: Pre-generated Vin_i disturbances signals with dt = 10e-5.
#### simulation_result
All the simulation results generated is saved here.


## Key Features
* **Run experiments and visualize**: Run `simulate_and_plot_all_cases.m` to reproduce experiments from the paper.

## Note
The 
