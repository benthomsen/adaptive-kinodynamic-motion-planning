clear; clc;

start = [2,2,0.1,0,0,0];
goal  = [98,98,0,0,0,0];

radius = 1;

state_limits = ...
    [0,100;
    0,100;
    -pi/4,pi/4;
    -10,10;
    -10,10;
    -2,2];

input_limits = ...
    [-25, 25;
    -25, 25];

state_dims = 6;
input_dims = 2;


[A, B] = linstate_tandem(start');

c = zeros(state_dims,1);
R = 100*diag([1,10]);

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

obstacles = [60,0,10,40;
             70,60,10,40];

disp('setting parameters');
rrt = rrtstar_v4(A, B, c, R, start', goal', 300);
disp('starting algorithm');

state_free = @(state)(ball_is_state_free_new(state, state_limits, obstacles, radius));
input_free = @(input)(is_input_free_new(input, input_limits));
sample_state = @()(ball_sample_free_states(state_limits, obstacles, radius ));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(plot_2d_nl(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
% display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(ball_plot_new(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
linstate = @(x_i)(linstate_tandem(x_i));

[T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display, linstate);

% [msgStr,msgId] = lastwarn;
% warnStruct = warning('off',msgId);
