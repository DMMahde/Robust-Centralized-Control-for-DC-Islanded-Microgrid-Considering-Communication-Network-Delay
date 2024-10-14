# Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
[Transaction of the Institute of Measurement and Control, Sage Journals, *November 25, 2019*](https://doi.org/10.1177/0142331219884804)

## Abstract
The integration of renewable energy resources to DC microgrid has captured the attention of the researchers in recent years. 
One of the active field of application of DC distribution is the islanded DC microgrid (DC ImG). 
The DC ImG present numerous challenges to researchers. Among many challenges, the regulation of voltage and stability of the system is indispensable to efficient operation. 
The voltage stability problem becomes more prominent when the system is exposed to disturbances and possess uncertainties in parameters. 
However, challenges can be overcome by skilful design of a robust controller for the system. 
Therefore, in this paper, an output-feedback based centralized robust control scheme is proposed. 
The proposed controller is designed to maintain good control performance in the presence of parametric uncertainties and exogenous disturbances. 
The uncertainties of the DC microgrid is modelled as a linear time-varying state-space system. 
The upper and the lower bounds of the time-varying parameters are determined by a Lebesque-measurable matrix. 
To attenuate the exogenous disturbances of the system $H_\infty$ based output-feedback controller is designed. 
The system stability is assured by the Lyapunov function candidate. The output-feedback controller needs only the voltage measurement; 
therefore, it requires less communication bandwidth as compared to the state-feedback. 
To obtain the controller parameters linear matrix inequality constraints are formulated and solved. 
The performance of the proposed controller is verified via simulations and compared with the existing schemes.

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
