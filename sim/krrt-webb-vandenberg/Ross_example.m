clear; clc;

start = [0,0,100,0.05,5,0];
goal  = [100,100,80,pi/4,5,0];

radius = 2;

state_limits = ...
    [-100,200;
    -100,200;
    1,200;
    -2*pi,2*pi;
    0,15;
    -0.25, 0.25];

sampling_limits = ...
    [-20,120;
    -20,120;
    60,120;
    -pi/2, pi/2
    0,10;
    -0.25, 0.25];

input_limits = ...
    [-pi, pi;
    -2.5, 2.5;
    -12, 12];

state_dims = 6;
input_dims = 3;


[A, B] = Ross_linstate(start');

c = zeros(state_dims,1);
R = 2*diag([2,1,1]);

% (center_x, center_y, center_z, width, length, height)
obstacles = [65,20,20,20,40,40;
             75,80,80,20,40,40];

disp('setting parameters');
rrt = rrtstar_v5(A, B, c, R, start', goal', 200, 0.05);
disp('starting algorithm');

state_free = @(state)(is_state_free_3d(state, state_limits, obstacles, radius));
input_free = @(input)(is_input_free_new(input, input_limits));
sample_state = @()(sample_free_states_smart(sampling_limits, state_limits, obstacles, radius ));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(plot_3d_nl(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
linstate = @(x_i)(Ross_linstate(x_i));

% [cost, times, states, u] = rrt.calc_numerical(start', goal', A, B);

[T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display, linstate);

% [msgStr,msgId] = lastwarn;
% warnStruct = warning('off',msgId);
