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

sim_path_case = '..\data\simulation_result\Predictor_TDS_Case_Load_Dist_tau';
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
% i = 5;
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
ctrl_Name = [{'Predictor-based'},{'H_\infty Controller'}];
tstart = 0-0.01; tend = Tf+0.01;


% figure('Name', 'FIGURE 7: TDS controller performance against load decrease at $PCC_5$', 'NumberTitle','off')

% subplot(2,2,1)
% xlabel('(a)'); 
% subplot(2,2,2)
% xlabel('(b)')

subplot(2,2,3);
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
    ax.YTick = 350:10:390;
end
if i == 5
    ax.YLim = [330, 405];
    ax.YTick = 330:10:400;
end
ax.XLabel.String = {'Time (s)'; '(c)'};
ax.YLabel.String = 'Voltage (V)';


if i == 1
    % Define zoomed-in region
    t_zoom_l = 0.4; t_zoom_u = 0.8;
    t_zoom_ta1 = t_ta1(t_ta1 >= t_zoom_l & t_ta1 <= t_zoom_u);
    t_zoom_ta2 = t_ta2(t_ta2 >= t_zoom_l & t_ta2 <= t_zoom_u);
    t_zoom_ta3 = t_ta3(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);

    % Vref = Vref(i);
    v5ms_zoom = Vdci_ta1(t_ta1 >= t_zoom_l & t_ta1 <= t_zoom_u);
    v10ms_zoom = Vdci_ta2(t_ta2 >= t_zoom_l & t_ta2 <= t_zoom_u);
    v20ms_zoom = Vdci_ta3(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);
    % vref_zoom = Vref(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);
    % vcent_zoom = Vdc1_Cent(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);

    % Add a dotted rectangle
    rect_position = [0.49, 375, 0.3, 10];
    rectangle('Position', rect_position, 'LineStyle', ':', 'LineWidth', 1.5);

    % Create inset axes
    inset_position = [0.15 0.15 0.15 0.12];  % [x, y, width, height] of the inset plot
    axes('Position', inset_position);      % Create smaller axes
    
    % % Plot the zoomed-in portion
    % plot(t_zoom, v5ms_zoom,   '--', 'Color', rgb_ref,    'LineWidth' ,2); hold on;
    % plot(t_zoom, v10ms_zoom,   ':',  'Color', rgb_lqr,    'LineWidth' ,2);
    % plot(t_zoom, v20ms_zoom, '-.', 'Color', rgb_dcent,  'LineWidth' ,2);
    % plot(t_zoom, vref,  '-',  'Color', rgb_output, 'LineWidth' ,2);
    
    plot(t_zoom_ta1,v5ms_zoom,'-.',t_zoom_ta2,v10ms_zoom,':',t_zoom_ta3,v20ms_zoom,'r',...
     t_zoom_ta3,Vref(i)*ones(size(t_zoom_ta3)),'y--','LineWidth',2);

    
    grid on;
    ax = gca;
    ax.FontSize = 10;
    ax.GridLineStyle = '--';
    ax.LineWidth = 1.5;
    ax.XLim = [0.45 0.8]
    ax.XTick = 0.5:.1:0.7;
    ax.YLim = [376.5 385]
    ax.YTick = 378:2:384;
    
    hold off;
end


if i == 5
    % Define zoomed-in region
    t_zoom_l = 0.72; t_zoom_u = 0.8;
    t_zoom_ta1 = t_ta1(t_ta1 >= t_zoom_l & t_ta1 <= t_zoom_u);
    t_zoom_ta2 = t_ta2(t_ta2 >= t_zoom_l & t_ta2 <= t_zoom_u);
    t_zoom_ta3 = t_ta3(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);

    % Vref = Vref(i);
    v5ms_zoom = Vdci_ta1(t_ta1 >= t_zoom_l & t_ta1 <= t_zoom_u);
    v10ms_zoom = Vdci_ta2(t_ta2 >= t_zoom_l & t_ta2 <= t_zoom_u);
    v20ms_zoom = Vdci_ta3(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);
    % vref_zoom = Vref(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);
    % vcent_zoom = Vdc1_Cent(t_ta3 >= t_zoom_l & t_ta3 <= t_zoom_u);
    
    % Add a dotted rectangle
    rect_position = [0.72, 375, 0.08, 26];
    rectangle('Position', rect_position, 'LineStyle', ':', 'LineWidth', 1.5);
      
    % Create inset axes
    inset_position = [0.2 0.2 0.07 0.12];  % [x, y, width, height] of the inset plot
    axes('Position', inset_position);      % Create smaller axes
    
    % % Plot the zoomed-in portion
    % plot(t_zoom, v5ms_zoom,   '--', 'Color', rgb_ref,    'LineWidth' ,2); hold on;
    % plot(t_zoom, v10ms_zoom,   ':',  'Color', rgb_lqr,    'LineWidth' ,2);
    % plot(t_zoom, v20ms_zoom, '-.', 'Color', rgb_dcent,  'LineWidth' ,2);
    % plot(t_zoom, vref,  '-',  'Color', rgb_output, 'LineWidth' ,2);
    
    plot(t_zoom_ta1,v5ms_zoom,'-.',t_zoom_ta2,v10ms_zoom,':',t_zoom_ta3,v20ms_zoom,'r',...
     t_zoom_ta3,Vref(i)*ones(size(t_zoom_ta3)),'y--','LineWidth',2);

    
    grid on;
    ax = gca;
    ax.FontSize = 10;
    ax.GridLineStyle = '--';
    ax.LineWidth = 1.5;
    ax.XLim = [0.72 0.8];
    ax.XTick = 0.73:0.02:0.79;
    ax.YLim = [372 405];
    ax.YTick = 380:10:400;
    
    

    hold off;
end






subplot(2,2,4);
plot(t_ta1,(Vref(i)*ones(size(t_ta1))).^2./(rli_ta1(i,:)*1e3),...
     'LineWidth',2);

grid on;
ax = gca;
ax.FontSize = 14;
ax.GridLineStyle = '--';
ax.LineWidth = 1.75;
ax.XLim = [0, 1];
ax.XTick =  0:0.1:1;

if i == 1
    ax.YLim = [2.24, 3.75];
    ax.YTick = [2.25:0.25:3.5];
end

if i == 5
    ax.YLim = [0.5, 3.3];
    ax.YTick = 1:0.5:3;
end
ax.XLabel.String = {'Time (s)'; '(d)'};
ax.YLabel.String = 'Load (kW)';

% end