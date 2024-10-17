% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%
%% ---- DGU/DC ImG Toplogical Configuration
% II. DYNAMIC MODELING OF DC IMG
% FIGURE 4. DC ImG consisting of six DGUs.

Dd = D;
N = 6;          % Nos. of DGUs

h1 = 1000;      % Integrator free parameter
h2 = h1; h3 = h1; h4 = h1; h5 = h1; h6 = h1; 

% DGU_1:  DGU_1 is connected to DGUs 2, 3 & 6
A11 = [ (-1/Ct(1))*sum([1/Rij(1,2),1/Rij(1,3),1/Rij(1,6)]), (1-Dd(1))/Ct(1), 0
                                          -(1-Dd(1))/Lt(1),    -Rt(1)/Lt(1), 0
                                                       -h1,               0, 0];
                   
A12 = diag([1/(Rij(1,2)*Ct(1)), 0, 0]);
A13 = diag([1/(Rij(1,3)*Ct(1)), 0, 0]);
A14 = diag([0, 0, 0]);
A15 = diag([0, 0, 0]);
A16 = diag([1/(Rij(1,6)*Ct(1)), 0, 0]);

% DGU_2:  DGU_2 is connected to DGUs 1 & 4
A22 = [(-1/Ct(2))*sum([1/Rij(2,1),1/Rij(2,4)]), (1-Dd(2))/Ct(2), 0 
                              -(1-Dd(2))/Lt(2),    -Rt(2)/Lt(2), 0
                                           -h2,               0, 0];
A21 = diag([1/(Rij(2,1)*Ct(2)), 0, 0]);
A23 = diag([0, 0, 0]);
A24 = diag([1/(Rij(2,4)*Ct(2)), 0, 0]);
A25 = diag([0, 0, 0]);
A26 = diag([0, 0, 0]);

% DGU_3:  DGU_3 is connected to DGUs 1 & 4
A33 = [(-1/Ct(3))*sum([1/Rij(3,1),1/Rij(3,4)]), (1-Dd(3))/Ct(3), 0 
                              -(1-Dd(3))/Lt(3),    -Rt(3)/Lt(3), 0
                                           -h3,               0, 0];
A31 = diag([1/(Rij(3,1)*Ct(3)), 0, 0]);
A32 = diag([0, 0, 0]);
A34 = diag([1/(Rij(3,4)*Ct(3)), 0, 0]);
A35 = diag([0, 0, 0]);
A36 = diag([0, 0, 0]);

% DGU_4:  DGU_4 is connected to DGUs 2, 3 & 5
A44 = [(-1/Ct(4))*sum([1/Rij(4,2),1/Rij(4,3),1/Rij(4,5)]), (1-Dd(4))/Ct(4), 0 
                              -(1-Dd(4))/Lt(4),    -Rt(4)/Lt(4), 0
                                           -h4,                0, 0];
A41 = diag([0, 0, 0]);
A42 = diag([1/(Rij(4,2)*Ct(4)), 0, 0]);
A43 = diag([1/(Rij(4,3)*Ct(4)), 0, 0]);
A45 = diag([1/(Rij(4,5)*Ct(4)), 0, 0]);
A46 = diag([0, 0, 0]);

% DGU_5:  DGU_5 is connected to DGUs 4 & 6
A55 = [(-1/Ct(5))*sum([1/Rij(5,4),1/Rij(5,6)]), (1-Dd(5))/Ct(5), 0 
                              -(1-Dd(5))/Lt(5),    -Rt(5)/Lt(5), 0
                                           -h5,               0, 0];
A51 = diag([0, 0, 0]);
A52 = diag([0, 0, 0]);
A53 = diag([0, 0, 0]);
A54 = diag([1/(Rij(5,4)*Ct(5)), 0, 0]);
A56 = diag([1/(Rij(5,6)*Ct(5)), 0, 0]);

% DGU_6:  DGU_6 is connected to DGUs 1 & 5
A66 = [(-1/Ct(6))*sum([1/Rij(6,1),1/Rij(6,5)]), (1-Dd(6))/Ct(6), 0 
                              -(1-Dd(6))/Lt(6),    -Rt(6)/Lt(6), 0
                                           -h6,               0, 0];
A61 = diag([1/(Rij(6,1)*Ct(6)), 0, 0]);
A62 = diag([0, 0, 0]);
A63 = diag([0, 0, 0]);
A64 = diag([0, 0, 0]);
A65 = diag([1/(Rij(6,5)*Ct(6)), 0, 0]);
                    
A = [A11 A12 A13 A14 A15 A16
     A21 A22 A23 A24 A25 A26
     A31 A32 A33 A34 A35 A35
     A41 A42 A43 A44 A45 A46
     A51 A52 A53 A54 A55 A56
     A61 A62 A63 A64 A65 A66];
  
B1 = [ -Vin(1)/(((1-Dd(1))^2)*RL(1)*Ct(1))
                  Vin(1)/((1-Dd(1))*Lt(1))
                                         0];
B2 = [ -Vin(2)/(((1-Dd(2))^2)*RL(2)*Ct(2))
                  Vin(2)/((1-Dd(2))*Lt(2))
                                         0];
B3 = [ -Vin(3)/(((1-Dd(3))^2)*RL(3)*Ct(3))
                  Vin(3)/((1-Dd(3))*Lt(3))
                                         0];                                     
B4 = [ -Vin(4)/(((1-Dd(4))^2)*RL(4)*Ct(4))
                  Vin(4)/((1-Dd(4))*Lt(4))
                                         0];
B5 = [ -Vin(5)/(((1-Dd(5))^2)*RL(5)*Ct(5))
                  Vin(5)/((1-Dd(5))*Lt(5))
                                         0];
B6 = [ -Vin(6)/(((1-Dd(6))^2)*RL(6)*Ct(6))
                  Vin(6)/((1-Dd(6))*Lt(6))
                                         0];
                                     
B = blkdiag(B1,B2,B3,B4,B5,B6);

D1 = [-1/Ct(1),       0
             0,  1/Lt(1)
             0,       0];
D2 = [-1/Ct(2),       0
             0,  1/Lt(2)
             0,       0];
D3 = [-1/Ct(3),       0
             0,  1/Lt(3)
             0,       0];
D4 = [-1/Ct(4),       0
             0,  1/Lt(4)
             0,       0];
         
D5 = [-1/Ct(5),       0
             0,  1/Lt(5)
             0,       0];
D6 = [-1/Ct(6),       0
             0,  1/Lt(6)
             0,       0];         
         
D = blkdiag(D1,D2,D3,D4,D5,D6);
