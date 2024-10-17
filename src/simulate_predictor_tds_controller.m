% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
% Simulation for Predictor Based Robust Controller

%% ---- Simulation time production + delay
set_current_path;

dt = 5*1e-5;                        % time step
Tf = 1;                             % final time (s)
t1=0:dt:Tf;                         % time for producing delay
[r1, c1] = size(t1);
ta = tau*ones(size(t1));            % delay padding
max(tau)
t = -max(ta):dt:Tf;                 % simulation time
[r, c] = size(t1);
nt = length(t);                     % size of time
Nd = ceil(max(ta)/dt);              % number of delay samples
% c = c + Nd
x = zeros(nx,nt+1);                 % initialization of the state vector
y = zeros(nx,nt+1);                 % initialization of the output vector
u = zeros(nu,nt+1);                 % initialization of the input vector

%% ---- State Initializations

x1 = [Vref(1)*ones(1,Nd+1);Ireff(1)*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state
x2 = [Vref(2)*ones(1,Nd+1);Ireff(2)*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state
x3 = [Vref(3)*ones(1,Nd+1);Ireff(3)*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state
x4 = [Vref(4)*ones(1,Nd+1);Ireff(4)*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state
x5 = [Vref(5)*ones(1,Nd+1);Ireff(5)*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state
x6 = [Vref(6)*ones(1,Nd+1);Ireff(6)*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state

if zero_ics
    x = 0*[x1;x2;x3;x4;x5;x6];
    disp('Zero initial condition')
end

if sim_vin_dist
    xi(:,1:Nd+1) = [0*ones(1,Nd+1);0*ones(1,Nd+1);0*ones(1,Nd+1)];   % initial state
    x = [xi;xi;xi;xi;xi;xi];
    
elseif ~sim_vin_dist
    x = 0*[x1;x2;x3;x4;x5;x6];
    
else
    disp("Choose appropriate 'sim_vin_dist' value")
end

%% ---- Vin Disturbances
load('..\data\vin_signal\VinSignals_1min_5e5');

if sim_vin_dist
    en_VinDist = 1;         % Enable Input Voltage disturbance
    disp('Vin disturbances added');
    
elseif ~sim_vin_dist
    en_VinDist = 0;         % Disable Input Voltage disturbance
    disp('Vin disturbances NOT added');
else
    disp("Choose appropriate 'sim_vin_dist' value")
end


a = VinDist.signals(1).values(r1:Nd,:)'-Vin(1);
vin1_tild = en_VinDist*[a,VinDist.signals(1).values(1:c1,:)' - Vin(1)];

% figure, plot(t1,VinDist.signals(1).values(1:c1,:));
% figure, plot(t,vin1_tild);

a = VinDist.signals(2).values(r1:Nd,:)'-Vin(2);
vin2_tild = en_VinDist*[a,VinDist.signals(2).values(1:c1,:)' - Vin(2)];
a = VinDist.signals(3).values(r1:Nd,:)'-Vin(3);
vin3_tild = en_VinDist*[a,VinDist.signals(3).values(1:c1,:)' - Vin(3)];
a = VinDist.signals(4).values(r1:Nd,:)'-Vin(4);
vin4_tild = en_VinDist*[a,VinDist.signals(4).values(1:c1,:)' - Vin(4)];
a = VinDist.signals(5).values(r1:Nd,:)'-Vin(5);
vin5_tild = en_VinDist*[a,VinDist.signals(5).values(1:c1,:)' - Vin(5)];
a = VinDist.signals(6).values(r1:Nd,:)'-Vin(6);
vin6_tild = en_VinDist*[a,VinDist.signals(6).values(1:c1,:)' - Vin(6)];



%% ---- Load Variations
% Pload
PLmax = Pload*1.35;                  % Uncertainity in Load
PLmax(1) = 3500;                     % Set to 3.5 kW
PLmin = Pload*0.70;                  % Uncertainity in Load
PLmin(5) = 1000;                     % Set to 1.0 kW
RLmax = Vout.^2./PLmax;
RLmin = Vout.^2./PLmin;


if sim_load_dist
    % Load increase at PCC 1
    rl1 = [RL(1)*ones(r1,Nd) RL(1)*ones(r1,floor(c1/2)) RLmax(1)*ones(r1,ceil(c1/2))];
    iL1_tild = [Vref(1)./RL(1)*ones(r1,Nd) Vref(1)./(RL(1)*ones(r1,floor(c1/2))) Vref(1)./(RLmax(1)*ones(r1,ceil(c1/2)))];
    % Load decrease at PCC 5
    rl5 = [RL(5)*ones(r1,Nd) RL(5)*ones(r1,floor(3*c1/4)) RLmin(5)*ones(r1,ceil(c1/4))];
    iL5_tild = [Vref(5)./RL(5)*ones(r1,Nd) Vref(5)./(RL(5)*ones(r1,floor(3*c1/4))) Vref(5)./(RLmin(5)*ones(r1,ceil(c1/4)))];
    disp('Load disturbances added');
    

elseif ~sim_load_dist
    rl1 = RL(1)*ones(size(t));
    iL1_tild = Vref(1)./RL(1)*ones(size(t));
    rl5 = RL(5)*ones(size(t));
    iL5_tild = Vref(5)./RL(5)*ones(size(t));
    disp('Load disturbances NOT added');
else
    disp("Choose appropriate 'sim_load_dist' value")
end

rl2 = RL(2)*ones(size(t));
rl3 = RL(3)*ones(size(t));
rl4 = RL(4)*ones(size(t));
rl6 = RL(6)*ones(size(t));

iL2_tild = Vref(2)./RL(2)*ones(size(t));
iL3_tild = Vref(3)./RL(3)*ones(size(t));
iL4_tild = Vref(4)./RL(4)*ones(size(t));
iL6_tild = Vref(6)./RL(6)*ones(size(t));

%% ---- Disturbances Vector Update
w1 = [iL1_tild;vin1_tild]; w2 = [iL2_tild;vin2_tild];
w3 = [iL3_tild;vin3_tild]; w4 = [iL4_tild;vin4_tild];
w5 = [iL5_tild;vin5_tild]; w6 = [iL6_tild;vin6_tild];
w = [w1;w2;w3;w4;w5;w6];

%% ---- Simulation

tic
for indx_t = Nd+1:nt-1
    P_cent = 100*(indx_t)/nt;
    fprintf('TDS Predictor %s, %d ms: ---> %0.1f %% \n',simulation_type ,tau*1e3, P_cent)
    
    % end
 
% Updates required for simulation    
B1 = [ -Vin(1)/(((1-Dd(1))^2)*rl1(indx_t)*Ct(1))
                        Vin(1)/((1-Dd(1))*Lt(1))
                                              0];
B2 = [ -Vin(2)/(((1-Dd(2))^2)*rl2(indx_t)*Ct(2))
                  Vin(2)/((1-Dd(2))*Lt(2))
                                         0];
B3 = [ -Vin(3)/(((1-Dd(3))^2)*rl3(indx_t)*Ct(3))
                  Vin(3)/((1-Dd(3))*Lt(3))
                                         0];                                     
B4 = [ -Vin(4)/(((1-Dd(4))^2)*rl4(indx_t)*Ct(4))
                  Vin(4)/((1-Dd(4))*Lt(4))
                                         0];
B5 = [ -Vin(5)/(((1-Dd(5))^2)*rl5(indx_t)*Ct(5))
                  Vin(5)/((1-Dd(5))*Lt(5))
                                         0];
B6 = [ -Vin(6)/(((1-Dd(6))^2)*rl6(indx_t)*Ct(6))
                  Vin(6)/((1-Dd(6))*Lt(6))
                                         0];
B = blkdiag(B1,B2,B3,B4,B5,B6);

    % Computing prediction-vector and controller (:,:,indx_t)
    Integral_term = zeros(nx,1);
    if indx_t <= Nd
       for j = 1:indx_t
           Integral_term = Integral_term+expm(A*(t(indx_t)-t(j)))* dt *B*u(:,j);
       end
    else
       for k = indx_t-Nd+1:indx_t
           Integral_term = Integral_term+expm(A*(t(indx_t)-t(k)))* dt *B*u(:,k);
       end
    end
    Prediction_vector(:,indx_t) = M*x(:,indx_t)+Integral_term;
    u(:,indx_t) = K*Prediction_vector(:,indx_t);
    
    % Euler method for discretization
    if indx_t <= Nd
        x(:,indx_t+1) = x(:,indx_t)+dt*( A*x(:,indx_t)+D*w(:,indx_t) );      
    else
%        x(:,indx_t+1) = x(:,indx_t)+dt*( A*x(:,indx_t)+B*u(indx_t-Nd)+D*w(:,indx_t) );
                   k1 = ( A*x(:,indx_t)  + B*u(:,indx_t-Nd)+D*w(:,indx_t) )*dt;
                   k2 = ( A*(x(:,indx_t) + k1/2)+B*u(:,indx_t-Nd)+D*w(:,indx_t) )*dt;
                   k3 = ( A*(x(:,indx_t) + k2/2)+B*u(:,indx_t-Nd)+D*w(:,indx_t) )*dt;
                   k4 = ( A*(x(:,indx_t) + k3)+B*u(:,indx_t-Nd)+D*w(:,indx_t) )*dt;
        x(:,indx_t+1) = x(:,indx_t) + (k1+2*k2+2*k3+k4)/6 ;        
    end
    
end

Elapsed_time_sec = toc
x=x(:,1:end);       % Dimension adjustment
u=u(:,1:end-2);     % Dimension adjustment
u = [u, u(:,end)];

%% ---- Save Results

% File_Name = simulation_name;        % for result saving
File_Name = ['..\data\simulation_result\', simulation_name];        % for result saving

if sim_vin_dist
    File_Name = [File_Name, '_Vin_Dist_tau_', num2str(tau*1e3), 'ms']
    save(File_Name,'x','t','u','Dd','RL','Vin',...
               'w','w1','w2','w3','w4','w5','w6','Vref',...
               'rl1','rl2','rl3','rl4','rl5','rl6',...
               'lmda','tau','h','dt','Tf','gama','K','Elapsed_time_sec'); 
end

if sim_load_dist
    File_Name = [File_Name, '_Load_Dist_tau_', num2str(tau*1e3), 'ms']
    save(File_Name,'x','t','u','Dd','RL','Vin',...
               'w','w1','w2','w3','w4','w5','w6','Vref',...
               'rl1','rl2','rl3','rl4','rl5','rl6',...
               'lmda','tau','h','dt','Tf','gama','K','Elapsed_time_sec');
end
