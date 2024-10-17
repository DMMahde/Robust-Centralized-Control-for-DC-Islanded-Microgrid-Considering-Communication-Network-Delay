% Robust Centralized Control for DC Islanded Microgrid Considering Communication Network Delay
%     Muhammad Mehdi(https://orcid.org/0000-0001-6519-7906), Chul-Hwan Kim, Muhammad Saad
%---------------------------------------------------------------------------------------------
% Published in: IEEE Access ( Volume: 8)
% Page(s): 77765 - 77778                  DOI: https://doi.org/10.1109/ACCESS.2020.2989777
% Date of Publication: 23 April 2020 
% Electronic ISSN: 2169-3536              Publisher: IEEE
%---------------------------------------------------------------------------------------------
%% ---- FIGUREs 6 & 7: subplots
set_current_path;

% clear variables; close all; clc;

sim_path_case = '..\data\simulation_result\Robust_TDS_Case_Load_Dist_tau';
load([sim_path_case, '_5ms']);

xi_ta1 = zeros(3,length(x),6);
x1 = [ x(1,:);  x(2,:);  x(3,:)];  xi_ta1(:,:,1)= x1;
x2 = [ x(4,:);  x(5,:);  x(6,:)];  xi_ta1(:,:,2)= x2;
x3 = [ x(7,:);  x(8,:);  x(9,:)];  xi_ta1(:,:,3)= x3;
x4 = [ x(10,:); x(11,:); x(12,:)]; xi_ta1(:,:,4)= x4;
x5 = [ x(13,:); x(14,:); x(15,:)]; xi_ta1(:,:,5)= x5;
x6 = [ x(16,:); x(17,:); x(18,:)]; xi_ta1(:,:,6)= x6;

wi_ta1 = zeros(2,length(w1),6);
wi_ta1(:,:,1) = w1;    wi_ta1(:,:,2) = w2;
wi_ta1(:,:,3) = w3;    wi_ta1(:,:,4) = w4;
wi_ta1(:,:,4) = w4;    wi_ta1(:,:,6) = w6;
rli_ta1 = [rl1;rl2;rl3;rl4;rl5;rl6;];

u_ta1 = u;    t_ta1 = t;    tau1 = tau;

clear x w w1 w2 w3 w4 w5 w6 u rl1 rl2 rl3 rl4 rl5 rl6 t tau;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load([sim_path_case, '_10ms']);

xi_ta2 = zeros(3,length(x),6);
x1 = [ x(1,:);  x(2,:);  x(3,:)];  xi_ta2(:,:,1)= x1;
x2 = [ x(4,:);  x(5,:);  x(6,:)];  xi_ta2(:,:,2)= x2;
x3 = [ x(7,:);  x(8,:);  x(9,:)];  xi_ta2(:,:,3)= x3;
x4 = [ x(10,:); x(11,:); x(12,:)]; xi_ta2(:,:,4)= x4;
x5 = [ x(13,:); x(14,:); x(15,:)]; xi_ta2(:,:,5)= x5;
x6 = [ x(16,:); x(17,:); x(18,:)]; xi_ta2(:,:,6)= x6;

wi_ta2 = zeros(2,length(w1),6);
wi_ta2(:,:,1) = w1;    wi_ta2(:,:,2) = w2;
wi_ta2(:,:,3) = w3;    wi_ta2(:,:,4) = w4;
wi_ta2(:,:,4) = w4;    wi_ta2(:,:,6) = w6;
rli_ta2 = [rl1;rl2;rl3;rl4;rl5;rl6;];

u_ta2 = u;   t_ta2 = t;  tau2 = tau;

clear x w w1 w2 w3 w4 w5 w6 u rl1 rl2 rl3 rl4 rl5 rl6 t tau;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load([sim_path_case, '_20ms']);

xi_ta3 = zeros(3,length(x),6);
x1 = [ x(1,:);  x(2,:);  x(3,:)];  xi_ta3(:,:,1)= x1;
x2 = [ x(4,:);  x(5,:);  x(6,:)];  xi_ta3(:,:,2)= x2;
x3 = [ x(7,:);  x(8,:);  x(9,:)];  xi_ta3(:,:,3)= x3;
x4 = [ x(10,:); x(11,:); x(12,:)]; xi_ta3(:,:,4)= x4;
x5 = [ x(13,:); x(14,:); x(15,:)]; xi_ta3(:,:,5)= x5;
x6 = [ x(16,:); x(17,:); x(18,:)]; xi_ta3(:,:,6)= x6;

wi_ta3 = zeros(2,length(w1),6);
wi_ta3(:,:,1) = w1;    wi_ta3(:,:,2) = w2;
wi_ta3(:,:,3) = w3;    wi_ta3(:,:,4) = w4;
wi_ta3(:,:,4) = w4;    wi_ta3(:,:,6) = w6;
rli_ta3 = [rl1;rl2;rl3;rl4;rl5;rl6;];

u_ta3 = u;   t_ta3 = t;  tau3 = tau;

% clear x w w1 w2 w3 w4 w5 w6 u rl1 rl2 rl3 rl4 rl5 rl6 t;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i = 1:6
% i = 1;
Di_bar_ta1 =  Dd(i);                  % nominal value of duty cycle
di_ta1 = Di_bar_ta1+u_ta1(i,:);       % u is small signal of duty cycle produced by controller
Iti_bar_ta1 = Vin(i)./(((1-Di_bar_ta1)^2).*rli_ta1(i,:)); % nominal value of current (first state)
Vdci_bar_ta1 = Vin(i)/(1-Di_bar_ta1);  % nominal value of voltage (second state)
Vdci_ta1 = Vdci_bar_ta1 + xi_ta1(1,:,i);       % x2 is small signal of voltage, DGU i
Iti_ta1 = Iti_bar_ta1 + xi_ta1(2,:,i);         % x1 is small signal of current, DGU i
Vini_ta1 = Vin(i) + wi_ta1(2,:,i);
%--------------------------------------------------------------------------
Di_bar_ta2 =  Dd(i);           % nominal value of duty cycle
di_ta2 = Di_bar_ta2 + u_ta2(i,:);       % u is small signal of duty cycle produced by controller
Iti_bar_ta2 = Vin(i)./(((1-Di_bar_ta2)^2).*rli_ta2(i,:)); % nominal value of current (first state)
Vdci_bar_ta2 = Vin(i)/(1-Di_bar_ta2);            % nominal value of voltage (second state)
Vdci_ta2 = Vdci_bar_ta2 + xi_ta2(1,:,i);       % x2 is small signal of voltage, DGU i
Iti_ta2 = Iti_bar_ta2 + xi_ta2(2,:,i);         % x1 is small signal of current, DGU i
Vini_ta2 = Vin(i) + wi_ta2(2,:,i);
%--------------------------------------------------------------------------
Di_bar_ta3 =  Dd(i);           % nominal value of duty cycle
di_ta3 = Di_bar_ta3 + u_ta3(i,:);       % u is small signal of duty cycle produced by controller
Iti_bar_ta3 = Vin(i)./(((1-Di_bar_ta3)^2).*rli_ta3(i,:)); % nominal value of current (first state)
Vdci_bar_ta3 = Vin(i)/(1-Di_bar_ta3);            % nominal value of voltage (second state)
Vdci_ta3 = Vdci_bar_ta3 + xi_ta3(1,:,i);       % x2 is small signal of voltage, DGU i
Iti_ta3 = Iti_bar_ta3 + xi_ta3(2,:,i);         % x1 is small signal of current, DGU i
Vini_ta3 = Vin(i) + wi_ta3(2,:,i);

Ireff = Vin./(((1-Dd).^2).*RL);
%------------------------------------------------------------------------

% close all;
% tstart = -tau; tend = Tf;
ctrl_Name = [{'H_\infty'},{'Controller'}];
tstart = 0-0.01; tend = Tf+0.01;
% i = 2;

% figure('Name', 'FIGURE 7: TDS controller performance against load decrease at $PCC_5$', 'NumberTitle','off')

% subplot(2,2,1)
% xlabel('(a)'); 
% subplot(2,2,3)
% xlabel('(c)')

% subplot(2,2,2);
plot(t_ta1,Vdci_ta1,'-.',t_ta2,Vdci_ta2,':',t_ta3,Vdci_ta3,'r',...
     t_ta3,Vref(i)*ones(size(t_ta3)),'y--','LineWidth',2);

lgd = legend(['\tau =  ',num2str(tau1*1e3),'   (ms)'],...
             ['\tau = ',num2str(tau2*1e3),'  (ms)'],...
             ['\tau =  ',num2str(tau3*1e3),' (ms)'],...
             ['V_',num2str(i),'ref:'], 'Location','southeast');
title(lgd,ctrl_Name);          

grid on;
ax = gca;
ax.FontSize = 14;
ax.GridLineStyle = '--';
ax.LineWidth = 1.75;
ax.XLim = [0, 1];
ax.XTick =  0:0.1:1;

if i == 1
    ax.YLim = [345, 395];
    ax.YTick = 350:20:390;
end
if i == 5
    ax.YLim = [330, 405];
    ax.YTick = 330:10:400;
end




ax.XLabel.String = '(b)';
ax.YLabel.String = 'Voltage (V)';


% 
% subplot(2,2,4);
% plot(t_ta1,(Vref(i)*ones(size(t_ta1))).^2./(rli_ta1(i,:)*1e3),...
%      'LineWidth',2);
% 
% grid on;
% ax = gca;
% ax.FontSize = 14;
% ax.GridLineStyle = '--';
% ax.LineWidth = 1.75;
% ax.XLim = [0, 1];
% ax.XTick =  0:0.1:1;
% ax.YLim = [0.5, 3.3];
% ax.YTick = 1:0.5:3;
% ax.XLabel.String = {'Time (s)'; '(d)'};
% ax.YLabel.String = 'Load (kW)';

% end