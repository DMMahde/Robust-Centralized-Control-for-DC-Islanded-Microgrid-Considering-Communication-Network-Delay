# Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
[IEEE Access, *Vol: 9*, *April 23, 2020*]([https://doi.org/10.1177/0142331219884804](https://doi.org/10.1109/ACCESS.2020.2989777))

## Abstract
In recent years, the application of renewable energy resources (RES) with DC output has increased, and RES integration as DC islanded microgrids (DC ImGs) has attracted the attention of many researchers. However, DC ImGs face many challenges, and voltage stability is extremely critical for efficient power distribution. This challenge becomes more prominent when exogenous disturbances, as well as time-delay, exist in the system mainly because of the communication network. In this study, we develop a mathematical model of the time-delay DC ImG. To compensate for the effect of the time-delay, three control strategies are introduced-stabilizing, robust, and robust-predictor. The controller's stability is guaranteed based on the Lyapunov-Krasovskii theorem, whereas for the exogenous disturbance rejection, the $\mathcal(L)_2$ -norm of the system is reduced. Furthermore, to obtain the proposed controllers' gains, linear-matrix-inequality constraints are formulated. The performances of the controllers are investigated through numerous simulations, and a detailed analysis is presented.

## Getting Started
[YALMIP](https://yalmip.github.io/) and [MOSEK](https://www.mosek.com/) tolbox for [MATLAB](https://www.mathworks.com/?s_tid=gn_logo) is required to run the code.
* Run `yalmiptest` to check correct installation of YALMIP.
* Run `mosekdiag` to check correct installation of MOSEK.

## Code Structure
* `Four_DGU_Output_Feedback_Uncertainty.m`: Main simulation file for the research paper.
* `Four_DGU_Uncertainty_data_5pct.m`: DC ImG nominal parameters and with 5% uncertainity radius.
* `DGU_parameters_Taxas_Inst.m`: Boost converter parameters calculated from Taxas Instruments Application 
 Report SLVA372C–November 2009–Revised January 2014.
* `LQR_Centralized.m`: Centralized LQR controller for comparison.
* `Decentralized_Four_DGU.m': Dentralized LQR controller for comparison.
* `Plot_Vt_ALL_Output_Feedback_mFile_Cent.m`: Plot of the case *Vref change*.
* `Plot_Load_Output_Feedback_mFile_Cent.m` : Plot of the case *Load disturbance*.
* `Plot_Vin_Dist_Four_DGU_Output_Feedback_Uncertainty.m`: Plot of the case *Vin disturbance*. 

## MATLAB Data Files
* `VinSingnals.mat`: Boost converters input voltage signals.
* `XOutput_Vt.mat`: DC ImG states, proposed output-feedback controller *Voltage tracking*.
* `XOutput_Load.mat`: DC ImG states, proposed output-feedback controller *Load disturbance*.
* `XOutput_Vin.mat`: DC ImG states, proposed output-feedback controller *Vin disturbances*.
* `Xdecent_Vt.mat`: DC ImG states, decentralized controller with *Voltage tracking*.
* `Xdecent_Load.mat`: DC ImG states, decentralized controller with *Load distrubance*.
* `Xdecent_Vin.mat`: DC ImG states, decentralized controller with *Vin disturbance*.
* `Xlqr_Vt.mat`: DC ImG states, centralized LQR controller with *Voltage tracking*.
* `Xlqr_Load.mat`: DC ImG states, centralized LQR controller with *Load distrubance*.
* `Xlqr_Vin.mat`: DC ImG states, centralized LQR controller with *Vin disturbance*.


## Key Features
* **Run experiments**: Run `Four_DGU_Output_Feedback_Uncertainty.m` to reproduce experiments from the paper.
* **Visualize results**: Run `Plot_xyz.m` to observe the results.

## Note
The comparision models of **LQR** and **Decentralized** controllers are not provided. Only the resuls of the simulation is provided here for plotting purposes
