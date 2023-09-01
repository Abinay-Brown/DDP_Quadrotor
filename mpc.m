% Horizon 
dt = 0.01;

m = 0.5;
g = 9.81;
Qbas = 1;
% Weight in Final State:
Q = diag([0.005, 0.005, 0.005, 1, 1, 1, 1,1,1, 200,200,200, Qbas]);
Q_f = 1*eye(13,13);
Q_f(1,1) = 800; 
Q_f(2,2) = 800; 
Q_f(3,3) = 800; 
Q_f(4,4) = 10; 
Q_f(5,5) = 10; 
Q_f(6,6) = 10; 
Q_f(7,7) = 0.00000001; 
Q_f(8,8) = 0.00000001; 
Q_f(9,9) = 0.00000001; 
Q_f(10,10) = 0.000000001; 
Q_f(11,11) = 0.000000001; 
Q_f(12,12) = 0.000000001;
Q_f(13,13) = Qbas;
% Weight in the Control:
R = 0.000005*eye(4,4);

% Initial Configuration:
xo = zeros(13,1);
xo(1,1) = -3;
xo(2,1) = -2;
xo(3,1) = -1;
uo = m*g*ones(4,1)/4;

% Target: 
p_target = zeros(13,1);
p_target(1,1) = 5;
p_target(2,1) = 3;
p_target(3,1) = 2;

% Obstacle Equations
h1_0 = (0-2.2)^2 + (0-2.2)^2 + (0-1)^2 - 1;
hx1_0 = [2*(0-2.2), 2*(0-2.2), 2*(0-1), 0, 0, 0, 0, 0, 0, 0, 0, 0];

h2_0 = (0)^2 + (0+0.2)^2 + (0)^2 - 1;
hx2_0 = [2*0, 2*(0+0.2), 2*0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

h3_0 = (0-3)^2 + (0)^2 + (0-0.5)^2 - 1;
hx3_0 = [2*(0-3), 2*0, 2*(0-0.5), 0, 0, 0, 0, 0, 0, 0, 0, 0];

h_0 = [h1_0, h2_0, h3_0];

%% MPC code
Horizon = 8/dt;
gamma = 1;
[x_ddp, u_ddp] = calcDDP(xo, uo, p_target, Q_f, Q, R, Horizon, dt, h_0, hx1_0, hx2_0, hx3_0, gamma);
%% Plotting the Result
time(1)=0;
for i= 2:Horizon
    time(i) =time(i-1) + dt;  
end
% quadrotor_visualize_w_obstacles(x_ddp, u_ddp, time);

disp(x_ddp(1:6,end));
% figure(1);
% u_traj = u_ddp;
%obsplot(x_ddp);
% figure(2);
%plot(time, x_ddp(6,:))
% x_traj = x_ddp;
% quadplot