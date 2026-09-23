% Automatic flight control of F8 aircraft using model predictive control
% and true system dynamics as model


clear all, close all, clc

SystemModel = 'F8';

figpath = ['../FIGURES/',SystemModel,'/'];mkdir(figpath)
datapath = ['../DATA/',SystemModel,'/'];mkdir(datapath)
addpath('../utils');

%% MPC
dt = 0.01;
Ts = 0.01;
Nvar = 3;

% Parameters MPC
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',100);
Duration = 6;                  % Run for 'Duration' time units
Ton = 0;                       % Time units when control is turned on
getMPCparams                   % MPC parameters

x0n=[0.1 0 0]';                % Initial condition

Results = struct('r', [], 'x',[], 'u', [], 't', [], 'xref', [], 'J', []);

%% Run MPC

% Prepare variables // Initialization
Nt       = (Duration/Ts)+1;
uopt0    = 0;
xhat     = x0n;
uopt     = uopt0.*ones(Nu,1);
xHistory = zeros(Nvar,Nt); xHistory(:,1) = xhat;
uHistory = zeros(1,Nt); uHistory(1)   = uopt(1);
tHistory = zeros(1,Nt); tHistory(1)   = 0;
rHistory = zeros(1,Nt);

% Start simulation
fprintf('Simulation started.  It might take a while...\n')
tic
for ct = 1:Duration/Ts 
    
    % Set references over optimization horizon
    tref = (ct:ct+N-1).*Ts;
    xref = [xrefFUN(tref); zeros(1,N); zeros(1,N)];
    
    % NMPC with full-state feedback
    COSTFUN = @(u) ObjectiveFCN(u,xhat,Ts,N,Nu,xref,uHistory(:,ct),[],diag(Q),R,Ru);
    CONSFUN = @(u) ConstraintFCN(u,uHistory(:,ct),xhat,Ts,N,LBo,UBo,LBdu,UBdu,[]);   
    uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);
    
    % Integrate system
    xhat = rk4u(@F8Sys,xhat,uopt(1),Ts/10,10,[],0); % Increase time resolution for simulation & keep control constant
    xHistory(:,ct+1) = xhat;
    uHistory(:,ct+1) = uopt(1);
    tHistory(:,ct+1) = ct*Ts;
    rHistory(:,ct+1) = xref(1,2);
    
end
tElapsed = toc
fprintf('Simulation finished!\n')


%% Collect results
Results.eval_time = tElapsed;
Results.xref = rHistory;
Results.x = xHistory;
Results.u = uHistory;
Results.t = tHistory;
Results.J = evalObjectiveFCN(Results.u,Results.x,Results.xref,diag(Q),R,Ru);

%% Show results
darkBackground = [0.08 0.10 0.14];
lightText = [0.92 0.95 0.98];
stateColors = [0.20 0.80 1.00; 1.00 0.58 0.20; 0.45 1.00 0.58];

figure('Color',darkBackground); hold on; box on
set(gca,'Color',darkBackground,'XColor',lightText,'YColor',lightText, ...
    'GridColor',[0.35 0.40 0.48],'LineWidth',1,'FontSize',12)
statePlots = plot(tHistory,xHistory','LineWidth',1.5);
for i = 1:Nvar
    statePlots(i).Color = stateColors(i,:);
end
referencePlot = plot(tHistory,rHistory,'--','Color',[1.00 0.78 0.45],'LineWidth',1.8);
xlabel('Time','Color',lightText)
ylabel('State','Color',lightText)
title('F8 MPC State Tracking','Color',lightText)
legend([statePlots; referencePlot],{'Angle of attack','Pitch angle', ...
    'Pitch rate','Reference'},'TextColor',lightText, ...
    'Color',darkBackground,'Location','best')

figure('Color',darkBackground); box on
set(gca,'Color',darkBackground,'XColor',lightText,'YColor',lightText, ...
    'GridColor',[0.35 0.40 0.48],'LineWidth',1,'FontSize',12)
controlPlot = plot(tHistory,uHistory,'Color',[1.00 0.58 0.20],'LineWidth',1.5);
xlabel('Time','Color',lightText)
ylabel('Control input','Color',lightText)
title('F8 MPC Control Input','Color',lightText)
legend(controlPlot,{'Elevator control input'},'TextColor',lightText, ...
    'Color',darkBackground,'Location','best')

figure('Color',darkBackground); box on
set(gca,'Color',darkBackground,'XColor',lightText,'YColor',lightText, ...
    'GridColor',[0.35 0.40 0.48],'LineWidth',1,'FontSize',12)
costPlot = plot(tHistory,Results.J,'Color',[0.70 1.00 0.78],'LineWidth',1.5);
xlabel('Time','Color',lightText)
ylabel('Cost','Color',lightText)
title('F8 MPC Objective Function','Color',lightText)
legend(costPlot,{'Cumulative objective'},'TextColor',lightText, ...
    'Color',darkBackground,'Location','best')
