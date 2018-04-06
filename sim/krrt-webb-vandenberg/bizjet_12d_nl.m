clear; clc;
%	DEFINITION OF THE STATE VECTOR (Fmodel)
%		x(1)    = 		Body-axis x inertial velocity, ub, m/s
%		x(2)    =		Body-axis y inertial velocity, vb, m/s
%		x(3)    =		Body-axis z inertial velocity, wb, m/s
%		x(4)    =		North position of center of mass WRT Earth, xe, m
%		x(5)    =		East position of center of mass WRT Earth, ye, m
%		x(6)    =		Negative of c.m. altitude WRT Earth, ze = -h, m
%		x(7)    =		Body-axis roll rate, pr, rad/s
%		x(8)    =		Body-axis pitch rate, qr, rad/s
%		x(9)    =		Body-axis yaw rate, rr,rad/s
%		x(10)   =		Roll angle of body WRT Earth, phir, rad
%		x(11)   =		Pitch angle of body WRT Earth, thetar, rad
%		x(12)   =		Yaw angle of body WRT Earth, psir, rad
%	DEFINITION OF THE CONTROL VECTOR (Gmodel)
%		u(1)    = 		Elevator, dEr, rad, positive: trailing edge down
%		u(2)    = 		Aileron, dAr, rad, positive: left trailing edge down
%		u(3)    = 		Rudder, dRr, rad, positive: trailing edge left
%		u(4)    = 		Throttle, dT, %
%		u(7)    =		Stabilator, dSr, rad

start = [0, 0, -3048, 95.766, 0, 7.1259, 0, 0, 0, 0, 0.0743, 0];
goal  = [1200, 0, -2948, 95.766, 0, 7.1259, 0, 0, 0, 0, 0.0743, 0];

u0 = [0; 0; 0; 0.1855; 0; 0; -0.0399];
start_mod = [start(4:6), start(1:3), start(7:end)]';
thresh	=	[.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1];
xj		=	[start_mod; u0];
ti      =   0;
xdotj		=	LinModel(ti,xj);
[dFdX,fac]	=	numjac('LinModel',ti,xj,xdotj,thresh,[],0);
Fmodel		=	dFdX(1:12,1:12)
Gmodel		=	dFdX(1:12,13:19)

c123 = Fmodel(:,1:3); c456 = Fmodel(:,4:6); Fmodel2 = [c456, c123, Fmodel(:,7:end)];
r123 = Fmodel2(1:3,:); r456 = Fmodel2(4:6,:); 
A = [r456; r123; Fmodel2(7:end,:)];

Gr123 = Gmodel(1:3,:);
Gr456 = Gmodel(4:6,:);
B_allu = [Gr456; Gr123; Gmodel(7:end,:)];

c = B_allu * u0; % trim condition
B = B_allu(:,1:4);


radius = 10;

state_limits = ...
    [-10000, 10000;
    -10000, 10000;
    -10000, -1000;
    80, 160;
    -5, 5;
    -25, 25;
    -2, 2;
    -2, 2;
    -1.5, 1.5;
    -pi/4, pi/4;
    -pi/4, pi/4;
    -4*pi, 4*pi];

input_limits = ...
    [ -pi/7, pi/7;
    -pi/8, pi/8;
    -pi/6, pi/6;
    0, 0.70] - u0(1:4);
%     [ -pi/7, pi/7;
%     -pi/8, pi/8;
%     -pi/6, pi/6;
%     0, 0.70;
%     -pi/20, pi/20] - u0;

sampling_limits = ...
    [0, 5100;
    -10, 10;
    -3200, -2800;
    95.76, 95.77;
    -1, 1;
    -15, 10;
    -0.01, 0.01;
    -0.01, 0.01;
    -0.1, 0.1;
    -pi/12, pi/12;
    -pi/8, pi/8;
    -pi/16, pi/16];

state_dims = 12;
input_dims = 4;

R = 100*diag([1,1,1,2]);

%obstacles = [40,10,20,80];
% obstacles = [60,0,10,20;
%              60,30,10,70;
%              30,0,10,70;
%              30,80,10,20];
% obstacles = [60,0,10,40;
%              70,60,10,40];
% obstacles = [60,0,1,1;
%              70,60,1,1];

% (center_x, center_y, center_z, width, length, height)
% obstacles = [65,20,20,20,40,40;
%              75,80,80,20,40,40];
obstacles = [65,20,20,1,1,1;
             75,80,80,1,1,1];


disp('setting parameters');
rrt = rrtstar_v4(A, B, c, R, start', goal', 800, 0.1);
disp('starting algorithm');

state_free = @(state)(is_state_free_3d(state, state_limits, obstacles, radius));
input_free = @(input)(is_input_free_new(input, input_limits));
sample_state = @()(sample_free_states_smart(sampling_limits, state_limits, obstacles, radius));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(plot_3d_nl(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
linstate = @(x_i)(linstate12d(x_i));
% displaycost = @(dc_scratch, obj, tree, parents, goal, goal_cost, goal_parent)(ball_plot_states_and_inputs(dc_scratch, obj, tree, parents, goal, goal_cost, goal_parent));

[cost, times, states, u] = rrt.calc_numerical(start', goal', A, B);

[T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display, linstate);

% [msgStr,msgId] = lastwarn;
% warnStruct = warning('off',msgId);
