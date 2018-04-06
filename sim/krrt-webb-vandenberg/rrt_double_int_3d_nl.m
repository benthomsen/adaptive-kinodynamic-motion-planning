clear; clc;

start = [2,2,10,8,0,-2];
goal  = [98,98,98,0,0,0];

radius = 1;

state_limits = ...
    [0,100;
    0,100;
    0,100;
    -10,10;
    -10,10;
    -10,10];

input_limits = ...
    [ -25, 25;
    -25, 25;
    -25, 25];

state_dims = 6;
input_dims = 3;

% A = [zeros(3), eye(3); zeros(3), zeros(3)];
A = [zeros(3), eye(3); -0.01*eye(3), -0.02*eye(3)];
% A = [0 0 1 0; 0 0 0 1; -0.01 0 -0.02 0; 0 -0.01 0 -0.02];
B = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]';
c = zeros(state_dims,1);

R = 20*diag([1,2,1]);

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
obstacles = [65,20,20,20,40,40;
             75,80,80,20,40,40];


disp('setting parameters');
rrt = rrtstar_v4(A, B, c, R, start', goal', 300);
disp('starting algorithm');

state_free = @(state)(is_state_free_3d(state, state_limits, obstacles, radius));
input_free = @(input)(is_input_free_new(input, input_limits));
sample_state = @()(ball_sample_free_states(state_limits, obstacles, radius ));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(plot_3d(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
% displaycost = @(dc_scratch, obj, tree, parents, goal, goal_cost, goal_parent)(ball_plot_states_and_inputs(dc_scratch, obj, tree, parents, goal, goal_cost, goal_parent));

[T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display, @(x_i) deal(A, B));
% [states, inputs] = rrt.evaluate_states_and_inputs(start', goal');

% [msgStr,msgId] = lastwarn;
% warnStruct = warning('off',msgId);
