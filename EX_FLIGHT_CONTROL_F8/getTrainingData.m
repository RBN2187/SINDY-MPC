% Execute this file to collect training data for model identification - Simulation of a measurment of the system

%% 1. INTITALIZE THE STATES AND SIMULATION TIMESTEPS
% Initialize the states and simulation timesteps
x0  = [0.3 0.1 -0.5];   % Initial states of the system
n   = length(x0);       % Number of variables/states
dt  = 0.01;             % Time step

% Simulation time
tspan =[0:dt:200];
Ntrain = (length(tspan)-1)/2 + 1;   % Training takes up half of the simulation time, the other half is used for validation

%% 2. GENERATE INPUT SIGNALS
switch InputSignalType
    case 'sine2'
        A = .05;
        forcing = @(x,t) [(A * (sin(0.7*t).*sin(.1*t).*sin(.2*t).*sin(.05*t)))];
        u = forcing(0,tspan);
        [t,x] = simulateF8RK4(@(t,x,u) F8Sys(t,x,u),tspan,x0,u);

    case 'sine3'
        forcing = @(x,t) (.5*sin(5*t).*sin(.5*t)+0.1).^3;
        u = forcing(0,tspan);
        [t,x] = simulateF8RK4(@(t,x,u) F8Sys(t,x,u),tspan,x0,u);

    case 'chirp'
        A = .5;
        forcing = @(x,t) A*chirp(t,[],max(tspan),0.1).^2;
        u = forcing(0,tspan);
        [t,x] = simulateF8RK4(@(t,x,u) F8Sys(t,x,u),tspan,x0,u);

    case 'noise'
        vareps = 0.01;
        Diff = @(t,x) [vareps; 0; 0];
        SDE = sde(@(t,x) F8Sys(t,x,0),Diff,'StartState',x0');
        rng(1,'twister')
        [x, t, u] = simByEuler(SDE, length(tspan), 'DeltaTime', dt);
        u = u';
        x = x(1:end-1,:); t = t(1:end-1);

    case 'prbs'
        A = 0.05236; 
        taulim = [1 8];
        states = [-0.5:0.25:0.5];
        Nswitch = 4000;
        forcing = @(x,t) A*prbs(taulim, Nswitch, states, t,0);

        u = zeros(size(tspan));
        for i = 1:length(tspan)
            u(i) = forcing(0,tspan(i));
        end
        [t,x] = simulateF8RK4(@(t,x,u) F8Sys(t,x,u),tspan,x0,u);
        figure,plot(tspan,u)
        
    case 'sphs'
        Pf = 10;% Fundamental period
        K = 2; 
        A = 0.1;
        forcing = @(x,t) A*sphs(Pf,K,t);
        u = forcing(0,tspan);
        [t,x] = simulateF8RK4(@(t,x,u) F8Sys(t,x,u),tspan,x0,u);
end

%% 3. SPLIT TRAINING AND VALIDATION DATA
figure;
plot(tspan,u);
title('Input Signal - Training and Validation')
legend('Input Signal')
xlabel('Time [s]')
ylabel('Input Signal [rad]')

figure;
plot(t,x,'LineWidth',1.5)
xlabel('Time')
ylabel('xi')
legend('angle of attack', 'pitch angle', 'pitch rate')
set(gca,'LineWidth',1, 'FontSize',14)
set(gcf,'Position',[100 100 300 200])
set(gcf,'PaperPositionMode','auto')

% xv is the validation data, x is the training data
xv = x(Ntrain+1:end,:);
x = x(1:Ntrain,:);

% uv is the validation input, u is the training input
uv = u(Ntrain+1:end);
u = u(1:Ntrain);

% tv is the validation time, t is the training time
tv = t(Ntrain+1:end);
t = t(1:Ntrain);

% tspanv is the validation time, tspan is the training time
tspanv = tspan(Ntrain+1:end);
tspan = tspan(1:Ntrain);

function [t,x] = simulateF8RK4(rhs,tspan,x0,u)
% Simulate the continuous-time system at the prescribed sample times.
t = tspan(:);
x = zeros(numel(t),numel(x0));
state = x0(:);
x(1,:) = state.';

for i = 1:numel(t)-1
    step = t(i+1)-t(i);
    state = rk4u(@(time,state,input,unused) rhs(time,state,input), ...
        state,u(i),step,1,[],[]);
    x(i+1,:) = state.';
end
end


